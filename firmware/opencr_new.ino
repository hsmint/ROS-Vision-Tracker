// OpenCR 팬/틸트 펌웨어: 읽기 / 쓰기 / 안전
//
// Raspberry Pi (ROS) -> USB Serial -> OpenCR -> DYNAMIXEL XM430 (1 Mbps)
// pan ID11 (홈 0틱, +/-180°), tilt ID12 (홈 2048틱, +/-135°)
//
// 쓰기: "PAN TILT\n"      홈 기준 목표 각도 (deg). 한계 밖이면 클램프
// 읽기: "POS PAN TILT\n"  홈 기준 현재 각도 (deg), 100Hz 자동 출력
//
// 안전:
//   - 목표 각도 한계 클램프, 이동 속도 고정 (60 deg/s)
//   - 모터 Bus Watchdog 500ms: OpenCR가 멈추면 모터가 스스로 정지
//   - 100Hz 점검: 통신·하드웨어 오류, 범위 이탈, 명령 유실 3회 연속 -> FAULT
//   - FAULT: 현재 위치 유지 (실패 시 토크 해제), 1초마다 자동 재초기화

#include <Dynamixel2Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

using namespace ControlTableItem;

Dynamixel2Arduino dxl(Serial3, 84);  // DYNAMIXEL 포트, 방향 제어 핀 84

// ============================================================
// 설정
// ============================================================

const uint8_t N = 2;                       // 0 = pan, 1 = tilt
const uint8_t ID[N]        = {11, 12};
const int32_t HOME_TICK[N] = {0, 2048};
const float   LIMIT_DEG[N] = {180, 135};
const float   MARGIN_DEG   = 2;            // 범위 이탈 판정 여유 (오버슈트)
const float   TICK_PER_DEG = 4096 / 360.0f;

const int32_t PROFILE_VEL = 44;            // x1.374 = 60 deg/s
const int32_t PROFILE_ACC = 10;            // x21.46 = 215 deg/s^2 (약 0.3초 가속)

const uint32_t PERIOD_MS   = 10;           // 100Hz
const uint32_t RETRY_MS    = 1000;
const uint8_t  MAX_FAILS   = 3;
const uint8_t  WATCHDOG    = 25;           // x20ms = 500ms
const uint16_t ADDR_WATCHDOG = 98;         // XM430은 1바이트 (라이브러리 정의 오류 우회)
const uint16_t ADDR_GOAL     = 116;

// 116~135를 한 번에 읽는다. 빈 주소가 없어 Sync Read 하나로 충분하다.
struct __attribute__((packed)) Fb {
  int32_t goal;
  uint16_t realtime_tick;
  uint8_t moving, moving_status;
  int16_t pwm, current;
  int32_t velocity, position;
};
static_assert(sizeof(Fb) == 20, "Fb must cover 116~135");

// ============================================================
// 상태
// ============================================================

int32_t anchor[N];   // 홈에 해당하는 실제 엔코더 틱
int32_t goal[N];     // 보낸 목표 (틱)
Fb fb[N];            // 마지막 Sync Read 결과

DYNAMIXEL::XELInfoSyncWrite_t sw_xels[N];
DYNAMIXEL::InfoSyncWriteInst_t sw = {};
DYNAMIXEL::XELInfoSyncRead_t sr_xels[N];
DYNAMIXEL::InfoSyncReadInst_t sr = {};
uint8_t sr_buf[128];

bool ready = false;
uint8_t fails = 0;
uint32_t tick_ms = 0, retry_ms = 0;
const char *last_error = nullptr;

// ============================================================
// 통신
// ============================================================

// 상태 패킷 오류에는 하드웨어 오류 경고(0x80)도 포함된다.
bool commOk() {
  return dxl.getLastLibErrCode() == DXL_LIB_OK && dxl.getLastStatusPacketError() == 0;
}

bool rd(uint8_t i, uint8_t item, int32_t &v) {
  v = dxl.readControlTableItem(item, ID[i], 30);
  return commOk();
}

bool wr(uint8_t i, uint8_t item, int32_t v) {
  return dxl.writeControlTableItem(item, ID[i], v, 30) && commOk();
}

bool setWatchdog(uint8_t i, uint8_t v) {
  return dxl.write(ID[i], ADDR_WATCHDOG, &v, 1, 30) && commOk();
}

void setupSync() {
  for (uint8_t i = 0; i < N; ++i) {
    sw_xels[i].id = ID[i];
    sw_xels[i].p_data = reinterpret_cast<uint8_t *>(&goal[i]);
    sr_xels[i].id = ID[i];
    sr_xels[i].p_recv_buf = reinterpret_cast<uint8_t *>(&fb[i]);
  }
  sw.addr = ADDR_GOAL;
  sw.addr_length = sizeof(int32_t);
  sw.p_xels = sw_xels;
  sw.xel_count = N;

  sr.packet.p_buf = sr_buf;
  sr.packet.buf_capacity = sizeof(sr_buf);
  sr.addr = ADDR_GOAL;
  sr.addr_length = sizeof(Fb);
  sr.p_xels = sr_xels;
  sr.xel_count = N;
  sr.is_info_changed = true;
}

// 두 축 목표를 패킷 하나로 보낸다. 응답이 없으므로 확인은 다음 Sync Read가 한다.
void sendGoal() {
  sw.is_info_changed = true;
  dxl.syncWrite(&sw);
}

bool readFb() {
  if (dxl.syncRead(&sr, 5) != N) return false;
  for (uint8_t i = 0; i < N; ++i) {
    if (sr_xels[i].error) return false;
  }
  return true;
}

float tickToDeg(uint8_t i, int32_t t) { return (t - anchor[i]) / TICK_PER_DEG; }

// ============================================================
// 초기화와 FAULT
// ============================================================

// Extended Position, 속도 기반 프로파일, 응답 지연 0으로 설정하고 현재 위치에서 토크를 켠다.
// 실패하면 원인 문자열, 성공하면 nullptr.
const char *initAxes() {
  for (uint8_t i = 0; i < N; ++i) {
    int32_t mode, drive, delay_us, p;
    if (!dxl.ping(ID[i])) return "no response";
    if (!rd(i, OPERATING_MODE, mode) || !rd(i, DRIVE_MODE, drive) ||
        !rd(i, RETURN_DELAY_TIME, delay_us)) return "config read / hardware error";

    // EEPROM 항목은 토크를 끈 뒤에만 바꿀 수 있다.
    if (mode != OP_EXTENDED_POSITION || (drive & 4) || delay_us != 0) {
      if (!dxl.torqueOff(ID[i]) || !wr(i, DRIVE_MODE, drive & ~4) ||
          !wr(i, OPERATING_MODE, OP_EXTENDED_POSITION) ||
          !wr(i, RETURN_DELAY_TIME, 0)) return "config write";
    }

    // 한 바퀴 안에서 가장 가까운 홈을 기준점으로 잡는다.
    if (!rd(i, PRESENT_POSITION, p)) return "position read";
    int32_t rel = ((p - HOME_TICK[i]) % 4096 + 4096) % 4096;
    if (rel > 2048) rel -= 4096;
    anchor[i] = p - rel;
    if (fabsf(tickToDeg(i, p)) > LIMIT_DEG[i] + MARGIN_DEG) return "outside range; move by hand";

    // 목표를 현재 위치로 먼저 써야 토크를 켤 때 튀지 않는다.
    goal[i] = p;
    if (!setWatchdog(i, 0) || !wr(i, PROFILE_ACCELERATION, PROFILE_ACC) ||
        !wr(i, PROFILE_VELOCITY, PROFILE_VEL) || !wr(i, GOAL_POSITION, p) ||
        !dxl.torqueOn(ID[i]) || !setWatchdog(i, WATCHDOG)) return "torque on";
  }
  return readFb() ? nullptr : "sync read";
}

// 현재 위치에서 정지한다. 확실하게 하려고 응답을 받는 개별 쓰기를 쓰고, 실패한 축은 토크를 끈다.
void fault(const char *why) {
  ready = false;
  retry_ms = millis();
  for (uint8_t i = 0; i < N; ++i) {
    int32_t p;
    if (!setWatchdog(i, 0) || !rd(i, PRESENT_POSITION, p) || !wr(i, GOAL_POSITION, p)) {
      dxl.torqueOff(ID[i]);
    }
  }
  last_error = why;
  Serial.print("FAULT ");
  Serial.println(why);
}

// ============================================================
// 읽기 / 쓰기
// ============================================================

// "POS PAN TILT"를 버퍼에 만든 뒤 한 번에 보낸다. (USB 패킷 1개, float printf 불필요)
void printPos() {
  char buf[32] = "POS";
  char *p = buf + 3;
  for (uint8_t i = 0; i < N; ++i) {
    long v = lroundf(tickToDeg(i, fb[i].position) * 100);
    *p++ = ' ';
    if (v < 0) {
      *p++ = '-';
      v = -v;
    }
    p += sprintf(p, "%ld.%02ld", v / 100, v % 100);
  }
  *p++ = '\n';
  Serial.write(buf, p - buf);
}

// "PAN TILT"를 읽어 한계 안으로 클램프한 뒤 보낸다.
void writePos(const char *line) {
  char *end0, *end1;
  float deg[N];
  deg[0] = strtof(line, &end0);
  deg[1] = strtof(end0, &end1);
  if (end0 == line || end1 == end0 || !isfinite(deg[0]) || !isfinite(deg[1])) {
    Serial.println("ERR use: PAN_DEG TILT_DEG");
    return;
  }
  if (!ready) {
    Serial.println("ERR not ready");
    return;
  }
  for (uint8_t i = 0; i < N; ++i) {
    goal[i] = anchor[i] + lroundf(constrain(deg[i], -LIMIT_DEG[i], LIMIT_DEG[i]) * TICK_PER_DEG);
  }
  sendGoal();
}

void readSerial() {
  static char line[48];
  static uint8_t used = 0;

  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line[used] = '\0';
      if (used) writePos(line);
      used = 0;
    } else if (used < sizeof(line) - 1) {
      line[used++] = c;
    }
  }
}

// ============================================================
// setup / loop
// ============================================================

void setup() {
  Serial.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
  setupSync();
  delay(1000);   // 모터 전원과 USB 안정화
}

void loop() {
  readSerial();

  const uint32_t now = millis();

  // FAULT 또는 시작 전: 1초마다 재초기화. 같은 오류는 한 번만 출력한다.
  if (!ready) {
    if (now - retry_ms < RETRY_MS) return;
    retry_ms = now;
    const char *error = initAxes();
    if (!error) {
      ready = true;
      fails = 0;
      last_error = nullptr;
      Serial.println("READY");
    } else if (error != last_error) {
      last_error = error;
      Serial.print("WAIT ");
      Serial.println(error);
    }
    return;
  }

  // 100Hz: Sync Read 한 번으로 점검하고 위치를 출력한다. 이 통신이 Bus Watchdog도 갱신한다.
  if (now - tick_ms < PERIOD_MS) return;
  tick_ms = now;

  bool ok = readFb();
  for (uint8_t i = 0; ok && i < N; ++i) {
    if (fabsf(tickToDeg(i, fb[i].position)) > LIMIT_DEG[i] + MARGIN_DEG) {
      fault("out of range");
      return;
    }
    ok = fb[i].goal == goal[i];   // 불일치 = 명령 유실 또는 Watchdog 만료
  }
  if (ok) {
    fails = 0;
    printPos();
    return;
  }
  sendGoal();   // 유실된 목표 재전송
  if (++fails >= MAX_FAILS) fault("comm lost / hardware error");
}
