// OpenCR 통합 제어 펌웨어 (OPENCR_CONTROL_V3_RAD_VELOCITY)
//
// 흐름: Raspberry Pi main.py -> USB Serial(115200) -> OpenCR -> DYNAMIXEL(1 Mbps)
// 모터: X축(좌우, pan)  ID11, 홈 0틱
//       Y축(상하, tilt) ID12, 홈 2048틱
//
// 명령:
//   v PAN TILT    ID11/ID12 속도 (rad/s), 0은 위치 유지, 500ms 갱신 없으면 정지
//   DX DY SPEED   현재 목표 기준 상대 이동 (deg, deg, deg/s)
//   h             홈 복귀 (Y축 -> X축 순서)
//   x             즉시 정지하고 위치 저장
//   off           토크 해제 (카메라를 먼저 받칠 것)
//   p             상태 출력
//   scan          모터 진단
//   auto on/off   부팅 시 홈 복귀 / 저장 위치 복원
//
// 이동 결과는 홈 기준 X +/-180°, Y +/-135° 안에 있어야 한다.
// 마지막 정지 위치는 EEPROM에 저장하고 부팅할 때 복원한다.

#include <Dynamixel2Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

using namespace ControlTableItem;

// ============================================================
// 하드웨어 설정
// ============================================================

Dynamixel2Arduino dxl(Serial3, 84);  // DYNAMIXEL 포트, 방향 제어 핀 84
const char CONTROL_BUILD_ID[] = "OPENCR_CONTROL_V3_RAD_VELOCITY";

// 내부 배열 순서는 Y축이 0번, X축이 1번이다. (입력 순서 X, Y와 반대)
const uint8_t AXIS_Y = 0;
const uint8_t AXIS_X = 1;
const uint8_t AXIS_COUNT = 2;

const uint8_t ids[AXIS_COUNT] = {12, 11};
const int32_t homes[AXIS_COUNT] = {2048, 0};          // Y=180°, X=0° 위치의 엔코더 값
const float angle_limits[AXIS_COUNT] = {135, 180};   // 홈 기준 허용 각도 (+/-deg)

// ============================================================
// 단위와 동작 기준값
// ============================================================

const int32_t TICKS_PER_REV = 4096;                // 모터 한 바퀴 = 4096틱
const float VELOCITY_UNIT_DEG_S = 1.374f;          // Profile Velocity 1 = 1.374 deg/s
const float VELOCITY_UNIT_RAD_S = VELOCITY_UNIT_DEG_S * PI / 180.0f;
const uint32_t VELOCITY_COMMAND_TIMEOUT_MS = 500;
const float ACCEL_UNIT_DEG_S2 = 21.4577f;          // Profile Acceleration 1 = 21.4577 deg/s^2
const float ACCEL_TIME_S = 0.3f;                   // 설정 속도까지 걸리는 시간
const int32_t VELOCITY_LIMIT_MAX = 1023;
const int32_t EXTENDED_POSITION_MAX = 1048575;     // Extended Position 모드의 목표 범위
const uint8_t WATCHDOG_500MS = 25;                 // Bus Watchdog 1 = 20ms
const uint16_t ADDR_BUS_WATCHDOG = 98;             // XM430: exactly ONE byte
const uint16_t ADDR_PROFILE_ACCELERATION = 108;    // Sync Write 시작 주소

const float RETURN_SPEED_DEG_S = 30;               // 홈 복귀와 위치 복원 속도
const uint32_t POLL_INTERVAL_MS = 100;             // 모터 상태 점검 주기
const uint32_t MOVE_TIMEOUT_BASE_MS = 15000;       // 이동 제한 시간의 기본값
const int32_t ARRIVAL_TOLERANCE_TICKS = 8;         // 약 0.7° 이내면 도착
const uint8_t SETTLE_COUNT = 5;                    // 연속 도착 판정 횟수 (약 0.5초)

// ============================================================
// 제어 상태
// ============================================================

int32_t targets[AXIS_COUNT];   // 현재 목표 위치 (엔코더 틱)
int32_t anchors[AXIS_COUNT];   // 홈에 해당하는 실제 엔코더 틱. 홈 기준 위치 = targets - anchors
bool configured[AXIS_COUNT] = {false, false};

bool active = false;     // 이동 중
bool sequence = false;   // 순차 이동(홈 복귀·위치 복원) 중
bool faulted = false;    // 오류로 정지. 재시작 전까지 유지된다.
const char *reason = "none";

// 두 축 동시 이동
bool dual_move = false;
bool velocity_stream = false;
int32_t stream_velocity[AXIS_COUNT] = {0, 0};  // signed motor units; Y, X
uint32_t velocity_command_ms = 0;
uint8_t dual_settled[AXIS_COUNT] = {0, 0};
float dual_speeds[AXIS_COUNT] = {5, 5};

// 순차 이동: Y축을 먼저 옮기고, 도착하면 X축을 옮긴다.
float stage_speed = RETURN_SPEED_DEG_S;
const char *sequence_name = "HOME";
int32_t sequence_ticks[AXIS_COUNT] = {0, 0};   // 홈 기준 목표 틱
uint8_t stage = 0;                             // 지금 움직이는 축
uint8_t settled = 0;
uint32_t stage_ms = 0;                         // 이동 시작 시각
uint32_t poll_ms = 0;                          // 마지막 점검 시각

// ============================================================
// EEPROM 저장 구조
// ============================================================

// 주소 0: 부팅 시 자동 홈 복귀 설정
bool auto_home = false;

struct BootSetting {
  uint32_t magic;
  uint8_t enabled;
  uint8_t inverse;
};

const uint32_t BOOT_MAGIC = 0x43414D32;

// 주소 16: 마지막 정지 위치 (홈 기준 틱)
struct SavedPosition {
  uint32_t magic;
  int32_t ticks[AXIS_COUNT];
  uint32_t check;
};

const int POSITION_ADDR = 16;
const uint32_t POSITION_MAGIC = 0x504F5331;
const int32_t RESTORE_TOLERANCE = 12;   // 약 1.05° 이내면 같은 위치로 본다.
bool saved_valid = false;
int32_t saved_ticks[AXIS_COUNT] = {0, 0};

// ============================================================
// 기본 보조 함수
// ============================================================

// 모터의 제어 테이블 항목 하나를 읽는다. 통신·상태 오류가 없을 때만 true.
bool readValue(uint8_t id, uint8_t item, int32_t &value) {
  // Dynamixel2Arduino 0.8.2 incorrectly declares this XM430 item as 2 bytes.
  // Read only address 98; 0xFF represents the signed watchdog error value -1.
  if (item == BUS_WATCHDOG) {
    uint8_t raw = 0;
    if (dxl.read(id, ADDR_BUS_WATCHDOG, 1, &raw, sizeof(raw), 30) != 1 ||
        dxl.getLastLibErrCode() != DXL_LIB_OK || dxl.getLastStatusPacketError() != 0) return false;
    value = raw == 0xFF ? -1 : raw;
    return true;
  }
  const int32_t result = dxl.readControlTableItem(item, id, 30);
  if (dxl.getLastLibErrCode() != DXL_LIB_OK || dxl.getLastStatusPacketError() != 0) return false;
  value = result;
  return true;
}

// 통신 성공뿐 아니라 실제 레지스터 값까지 확인한다. 위치는 signed 32-bit 틱으로 쓴다.
bool verifyValue(uint8_t id, uint8_t item, int32_t expected) {
  int32_t actual;
  return readValue(id, item, actual) && actual == expected;
}

bool writeValue(uint8_t id, uint8_t item, int32_t value) {
  if (item == BUS_WATCHDOG) {
    if (value < 0 || value > 127) return false;
    const uint8_t raw = static_cast<uint8_t>(value);
    const bool written = dxl.write(id, ADDR_BUS_WATCHDOG, &raw, sizeof(raw), 30);
    if (!written || dxl.getLastLibErrCode() != DXL_LIB_OK || dxl.getLastStatusPacketError() != 0) {
      Serial.print("WATCHDOG_WRITE_ERROR id=");
      Serial.print(id);
      Serial.print(" lib=");
      Serial.print(dxl.getLastLibErrCode());
      Serial.print(" status=");
      Serial.println(dxl.getLastStatusPacketError());
      return false;
    }
    int32_t actual;
    if (!readValue(id, BUS_WATCHDOG, actual)) {
      Serial.print("WATCHDOG_READ_ERROR id=");
      Serial.println(id);
      return false;
    }
    if (actual != value) {
      Serial.print("WATCHDOG_VERIFY_ERROR id=");
      Serial.print(id);
      Serial.print(" expected=");
      Serial.print(value);
      Serial.print(" actual=");
      Serial.println(actual);
      return false;
    }
    return true;
  }
  if (!dxl.writeControlTableItem(item, id, value, 30) ||
      dxl.getLastLibErrCode() != DXL_LIB_OK || dxl.getLastStatusPacketError() != 0) return false;
  return verifyValue(id, item, value);
}

bool writePosition(uint8_t id, int32_t ticks) {
  return ticks >= -EXTENDED_POSITION_MAX && ticks <= EXTENDED_POSITION_MAX &&
         writeValue(id, GOAL_POSITION, ticks);
}

// 한 바퀴 경계를 고려해, 현재 위치에서 가장 가까운 홈까지의 차이(틱)를 구한다.
// 케이블이 몇 바퀴 감겼는지는 알 수 없다.
int32_t nearestError(int32_t position, int32_t home) {
  int32_t e = (home - position % TICKS_PER_REV) % TICKS_PER_REV;
  if (e > TICKS_PER_REV / 2) e -= TICKS_PER_REV;
  if (e < -TICKS_PER_REV / 2) e += TICKS_PER_REV;
  return e;
}

int32_t degToTicks(float deg) {
  return lroundf(deg * TICKS_PER_REV / 360.0f);
}

float ticksToDeg(int32_t ticks) {
  return ticks * 360.0f / TICKS_PER_REV;
}

// 허용 각도를 틱으로 바꾼다. X축 180° = 2048틱, Y축 135° = 1536틱.
int32_t limitTicks(uint8_t i) {
  return degToTicks(angle_limits[i]);
}

// 홈 기준 현재 목표 위치 (틱)
int32_t relativeTarget(uint8_t i) {
  return targets[i] - anchors[i];
}

// deg/s를 Profile Velocity 단위로 바꾼다. (내림)
int32_t speedToProfile(float speed) {
  return static_cast<int32_t>(floorf(speed / VELOCITY_UNIT_DEG_S));
}

// 모든 이동에서 동일한 0.3초 가속 기준을 사용한다. 0은 무한 가속이므로 제외한다.
int32_t accelerationForProfile(int32_t profile) {
  return constrain(static_cast<int32_t>(ceilf(profile * VELOCITY_UNIT_DEG_S /
                                            (ACCEL_UNIT_DEG_S2 * ACCEL_TIME_S))), 1, 32767);
}

// 허용 범위 끝에서 끝까지 이동할 시간 + 15초
uint32_t moveTimeoutMs(uint8_t i, float speed) {
  return MOVE_TIMEOUT_BASE_MS + static_cast<uint32_t>(ceilf(1000 * 2 * angle_limits[i] / speed));
}

// 목표 근처에서 거의 멈춰 있으면 true
bool isAtTarget(uint8_t i, int32_t position, int32_t velocity) {
  return llabs(static_cast<int64_t>(position) - targets[i]) <= ARRIVAL_TOLERANCE_TICKS && abs(velocity) <= 1;
}

// "X=.. Y=.." 형식으로 두 축 각도를 출력한다.
void printXY(int32_t x_ticks, int32_t y_ticks) {
  Serial.print("X=");
  Serial.print(ticksToDeg(x_ticks), 2);
  Serial.print(" Y=");
  Serial.print(ticksToDeg(y_ticks), 2);
}

void skipSpaces(const char *&text) {
  while (isspace(static_cast<unsigned char>(*text))) ++text;
}

// ============================================================
// 위치 저장 (EEPROM)
// ============================================================

// 저장 중 전원이 꺼져 값이 깨졌는지 확인하는 검사값
uint32_t positionCheck(const SavedPosition &s) {
  return ~(s.magic ^ static_cast<uint32_t>(s.ticks[0]) * 2654435761u ^ static_cast<uint32_t>(s.ticks[1]));
}

// EEPROM의 마지막 위치를 읽는다. 손상됐거나 범위를 벗어나면 쓰지 않는다.
void loadPosition() {
  SavedPosition s;
  EEPROM.get(POSITION_ADDR, s);
  saved_valid = s.magic == POSITION_MAGIC &&
                s.check == positionCheck(s) &&
                abs(s.ticks[AXIS_Y]) <= limitTicks(AXIS_Y) &&
                abs(s.ticks[AXIS_X]) <= limitTicks(AXIS_X);
  if (saved_valid) {
    saved_ticks[AXIS_Y] = s.ticks[AXIS_Y];
    saved_ticks[AXIS_X] = s.ticks[AXIS_X];
  }
}

// 두 축의 현재 목표를 EEPROM에 저장한다. 이미 저장된 값과 같으면 쓰지 않는다.
void savePosition() {
  if (!configured[AXIS_Y] || !configured[AXIS_X]) return;

  SavedPosition s = {POSITION_MAGIC, {relativeTarget(AXIS_Y), relativeTarget(AXIS_X)}, 0};
  s.check = positionCheck(s);
  if (saved_valid && s.ticks[AXIS_Y] == saved_ticks[AXIS_Y] && s.ticks[AXIS_X] == saved_ticks[AXIS_X]) return;

  EEPROM.put(POSITION_ADDR, s);
  SavedPosition verify;
  EEPROM.get(POSITION_ADDR, verify);
  if (memcmp(&verify, &s, sizeof(s))) {
    Serial.println("WARNING: position save failed.");
    return;
  }

  saved_valid = true;
  saved_ticks[AXIS_Y] = s.ticks[AXIS_Y];
  saved_ticks[AXIS_X] = s.ticks[AXIS_X];
  Serial.print("SAVED: ");
  printXY(s.ticks[AXIS_X], s.ticks[AXIS_Y]);
  Serial.println(" deg from home.");
}

// ============================================================
// 안전 함수
// ============================================================

// 진행 중인 이동을 취소하고, 토크가 켜진 축은 현재 위치를 새 목표로 잡아 그 자리에 멈춘다.
// 위치 유지에 실패하면 토크를 끈다.
bool hold() {
  active = false;
  sequence = false;
  dual_move = false;
  velocity_stream = false;
  stream_velocity[AXIS_Y] = stream_velocity[AXIS_X] = 0;

  bool ok = true;
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (!configured[i]) continue;

    int32_t p, torque;
    if (!readValue(ids[i], TORQUE_ENABLE, torque)) {
      Serial.println("WARNING: torque status unreadable; support camera.");
      ok = false;
      continue;
    }
    if (!torque) continue;

    if (!readValue(ids[i], PRESENT_POSITION, p) || !writePosition(ids[i], p)) {
      dxl.torqueOff(ids[i]);
      ok = false;
      Serial.println("WARNING: hold failed; torque off requested; support camera.");
    } else {
      targets[i] = p;
    }
  }
  return ok;
}

// 작업 성공 여부를 검사한다. 실패하면 원인을 기록하고 FAULT 상태로 정지한다.
bool require(bool ok, const char *why) {
  if (!ok) {
    reason = why;
    faulted = true;
    hold();
    Serial.print("FAULT: ");
    Serial.println(reason);
  }
  return ok;
}

// FAULT 상태이거나 이동 중이면 새 이동 요청을 거부한다.
bool readyForMove() {
  if (faulted) {
    Serial.print("FAULT: ");
    Serial.println(reason);
    return false;
  }
  if (active) {
    Serial.println("BUSY: send x to stop first.");
    return false;
  }
  return true;
}

// ============================================================
// 상태 출력 (p)
// ============================================================

void printStatus() {
  Serial.print("FIRMWARE=");
  Serial.println(CONTROL_BUILD_ID);

  Serial.print("STATE=");
  Serial.print(faulted ? "FAULT" : active ? "MOVING" : "IDLE");
  Serial.print(" reason=");
  Serial.print(reason);
  Serial.print(" return_speed_deg_s=");
  Serial.print(RETURN_SPEED_DEG_S);
  Serial.print(" auto_home=");
  Serial.println(auto_home ? "on" : "off");

  Serial.print("SAVED_POSITION=");
  if (saved_valid) {
    printXY(saved_ticks[AXIS_X], saved_ticks[AXIS_Y]);
    Serial.println(" deg");
  } else {
    Serial.println("none");
  }

  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    int32_t p, torque, error, cap, goal, velocity, profile, acceleration;
    Serial.print(i == AXIS_Y ? "tilt ID12 " : "pan ID11 ");
    if (!readValue(ids[i], PRESENT_POSITION, p) ||
        !readValue(ids[i], TORQUE_ENABLE, torque) ||
        !readValue(ids[i], HARDWARE_ERROR_STATUS, error) ||
        !readValue(ids[i], VELOCITY_LIMIT, cap) ||
        !readValue(ids[i], GOAL_POSITION, goal) ||
        !readValue(ids[i], PRESENT_VELOCITY, velocity) ||
        !readValue(ids[i], PROFILE_VELOCITY, profile) ||
        !readValue(ids[i], PROFILE_ACCELERATION, acceleration)) {
      Serial.println("READ_ERROR");
      continue;
    }

    // 준비 전이면 가장 가까운 홈을 기준으로 계산한다.
    const double home_ticks = configured[i] ? static_cast<double>(p) - anchors[i]
                                            : -nearestError(p, homes[i]);
    Serial.print("ticks=");
    Serial.print(p);
    Serial.print(" home_deg=");
    Serial.print(home_ticks * 360.0 / TICKS_PER_REV, 2);
    Serial.print(" goal_ticks=");
    Serial.print(goal);
    Serial.print(" velocity_deg_s=");
    Serial.print(velocity * VELOCITY_UNIT_DEG_S, 3);
    Serial.print(" home_rad=");
    Serial.print(home_ticks * 2.0 * PI / TICKS_PER_REV, 6);
    Serial.print(" velocity_rad_s=");
    Serial.print(velocity * VELOCITY_UNIT_RAD_S, 6);
    Serial.print(" profile_velocity_deg_s=");
    Serial.print(profile * VELOCITY_UNIT_DEG_S, 3);
    // XM430에는 Present Acceleration 항목이 없다. 설정된 프로파일 가속도를 출력한다.
    Serial.print(" profile_acceleration_deg_s2=");
    Serial.print(acceleration * ACCEL_UNIT_DEG_S2, 3);
    Serial.print(" torque=");
    Serial.print(torque);
    Serial.print(" hardware_error=");
    Serial.print(error);
    Serial.print(" velocity_limit_deg_s=");
    Serial.println(cap * VELOCITY_UNIT_DEG_S, 3);
  }
}

// ============================================================
// 축 준비
// ============================================================

// 축 i의 모델·오류·설정을 검사하고 Extended Position 모드로 준비한다.
// 처음 준비할 때 홈 기준점(anchors)을 정하고, 현재 위치를 초기 목표로 설정한다.
bool prepareAxis(uint8_t i) {
  const uint8_t id = ids[i];

  // 1. 연결과 모델 확인
  if (!require(dxl.ping(id), "ping") ||
      !require(dxl.getModelNumber(id) == XM430_W350, "model")) return false;

  // 2. 설정 점검
  int32_t offset, error, p, mode, drive, torque;
  if (!require(readValue(id, TORQUE_ENABLE, torque), "torque read")) return false;
  if (!require(readValue(id, HOMING_OFFSET, offset) && offset == 0, "home offset changed") ||
      !require(readValue(id, HARDWARE_ERROR_STATUS, error) && error == 0, "hardware error") ||
      !require(readValue(id, DRIVE_MODE, drive), "drive mode read") ||
      !require(readValue(id, OPERATING_MODE, mode), "mode read")) return false;

  // 3. EEPROM 설정은 토크를 끈 뒤 변경한다. Drive Mode bit 2만 지워
  //    속도 기반 프로파일을 선택하고 회전 방향 등 나머지 설정은 보존한다.
  if ((drive & 4) != 0 || mode != OP_EXTENDED_POSITION) {
    if (!require(writeValue(id, TORQUE_ENABLE, 0), "torque off")) return false;
    torque = 0;
    configured[i] = false;
    if ((drive & 4) != 0) {
      if (!require(writeValue(id, DRIVE_MODE, drive & ~4), "velocity profile setup")) return false;
      Serial.print("CONFIG: ID");
      Serial.print(id);
      Serial.println(" velocity-based profile enabled.");
    }
    if (mode != OP_EXTENDED_POSITION &&
        !require(writeValue(id, OPERATING_MODE, OP_EXTENDED_POSITION), "extended position mode")) return false;
  }

  // 4. 현재 위치 읽기
  if (!require(readValue(id, PRESENT_POSITION, p), "position read")) return false;

  // 5. 토크를 유지 중이면 마지막 목표를 상대 이동의 기준으로 이어 쓴다.
  //    그래야 상대 이동을 반복해도 오차가 쌓이지 않는다. 아니면 현재 위치가 기준이다.
  const bool keep_target = configured[i] && torque;

  // 6. 홈 기준점 계산 (처음 준비할 때만)
  //    가장 가까운 홈을 고르되, 저장 위치가 있으면 그 위치와 가장 가까운 바퀴를 고른다.
  if (!configured[i]) {
    int32_t rel = -nearestError(p, homes[i]);
    if (saved_valid) rel += TICKS_PER_REV * lroundf((saved_ticks[i] - rel) / 4096.0f);
    if (abs(rel) > limitTicks(i)) {
      Serial.println("REJECTED: current position outside home range; support and reposition with off.");
      return false;
    }
    anchors[i] = p - rel;
  }
  if (!keep_target) targets[i] = p;

  // 7. 감시 타이머 해제, 가장 완만한 가속도, 목표를 기준 위치로 (토크를 켜도 튀지 않게)
  if (!require(writeValue(id, BUS_WATCHDOG, 0), "watchdog clear") ||
      !require(writeValue(id, PROFILE_ACCELERATION, 1), "acceleration") ||
      !require(writePosition(id, targets[i]), "initial hold target")) return false;

  configured[i] = true;
  return true;
}

// ============================================================
// 한 축 이동 (홈 복귀·위치 복원에서 사용)
// ============================================================

// 축 i를 홈 기준 rel 틱 위치로 speed(deg/s)로 이동시킨다.
// 명령만 보내고 바로 끝나며, 도착 판정은 loop()가 한다.
bool moveAxis(uint8_t i, int32_t rel, float speed) {
  const uint8_t id = ids[i];

  int32_t p, cap;
  if (!require(readValue(id, PRESENT_POSITION, p), "position read") ||
      !require(readValue(id, VELOCITY_LIMIT, cap) && cap > 0 && cap <= VELOCITY_LIMIT_MAX, "velocity limit")) return false;

  const int32_t profile = speedToProfile(speed);
  if (profile < 1 || profile > cap) {
    Serial.println("REJECTED: speed outside motor limit.");
    return false;
  }

  const int64_t target = static_cast<int64_t>(anchors[i]) + rel;
  if (!require(target >= -EXTENDED_POSITION_MAX && target <= EXTENDED_POSITION_MAX, "extended position range")) return false;

  // 목표를 현재 위치로 먼저 쓴 뒤 토크를 켜야 튀지 않는다.
  if (!require(writeValue(id, PROFILE_ACCELERATION, accelerationForProfile(profile)), "profile acceleration") ||
      !require(writeValue(id, PROFILE_VELOCITY, profile), "profile velocity") ||
      !require(writePosition(id, p), "stage hold") ||
      !require(dxl.torqueOn(id), "torque on") ||
      !require(writeValue(id, BUS_WATCHDOG, WATCHDOG_500MS), "watchdog") ||
      !require(writePosition(id, static_cast<int32_t>(target)), "target write")) return false;

  targets[i] = target;
  stage = i;
  stage_ms = millis();
  settled = 0;
  active = true;
  stage_speed = speed;

  Serial.print("MOVING ");
  Serial.print(i == AXIS_Y ? "tilt " : "pan ");
  Serial.print(ticksToDeg(rel), 2);
  Serial.print(" deg from home at ");
  Serial.print(profile * VELOCITY_UNIT_DEG_S, 2);
  Serial.println(" deg/s.");
  return true;
}

// ============================================================
// 입력 해석
// ============================================================

// 부호 있는 십진수 하나를 읽는다. 동적 메모리를 쓰지 않는다.
// text는 읽은 만큼 앞으로 이동한 상태로 돌아간다.
bool readDecimal(const char *&text, float &value) {
  skipSpaces(text);

  bool negative = false;
  if (*text == '+' || *text == '-') {
    negative = *text == '-';
    ++text;
  }

  bool digit = false;
  double number = 0;
  while (*text >= '0' && *text <= '9') {
    digit = true;
    number = number * 10 + (*text++ - '0');
    if (number > 1000000) return false;
  }

  if (*text == '.') {
    ++text;
    double place = 0.1;
    while (*text >= '0' && *text <= '9') {
      digit = true;
      number += (*text++ - '0') * place;
      place *= 0.1;
    }
  }

  // 숫자가 없거나, 숫자 뒤에 공백이 아닌 글자가 붙어 있으면 실패
  if (!digit || (*text && !isspace(static_cast<unsigned char>(*text)))) return false;
  value = static_cast<float>(negative ? -number : number);
  return isfinite(value);
}

// "X변화량 Y변화량 속도"를 읽는다. 속도는 두 축에 같게 적용한다.
// 결과: values = {dX, dY, X속도, Y속도}
// 이동 후 위치가 홈 기준 범위 안인지는 moveBoth()에서 검사한다.
bool parseCommonMove(const char *text, float values[4]) {
  float parsed[3];
  for (uint8_t i = 0; i < 3; ++i) {
    if (!readDecimal(text, parsed[i])) return false;
  }
  skipSpaces(text);

  const float dx = parsed[0], dy = parsed[1], speed = parsed[2];
  if (*text) return false;   // 뒤에 다른 글자가 남아 있음
  if (fabsf(dx) > 2 * angle_limits[AXIS_X] || fabsf(dy) > 2 * angle_limits[AXIS_Y]) return false;
  if (speed < VELOCITY_UNIT_DEG_S) return false;   // 상한은 이동 전에 각 모터의 Velocity Limit으로 검사한다.

  values[0] = dx;
  values[1] = dy;
  values[2] = values[3] = speed;
  return true;
}

// ROS Twist angular.z -> PAN(ID11), angular.y -> TILT(ID12), both rad/s.
bool parseVelocity(const char *text, float &pan, float &tilt) {
  if (!readDecimal(text, pan) || !readDecimal(text, tilt)) return false;
  skipSpaces(text);
  return !*text && fabsf(pan) <= VELOCITY_LIMIT_MAX * VELOCITY_UNIT_RAD_S &&
                  fabsf(tilt) <= VELOCITY_LIMIT_MAX * VELOCITY_UNIT_RAD_S;
}

// Round toward zero: never exceed the requested speed; sub-unit speeds hold.
int32_t radiansToVelocity(float speed) {
  return static_cast<int32_t>(speed / VELOCITY_UNIT_RAD_S);
}

// Bounded velocity jogging in Extended Position mode. Magnitude sets profile
// speed, sign selects the travel boundary. The motor decelerates at that boundary
// and holds position there. No EEPROM mode switches on streamed commands.
void commandVelocity(float pan, float tilt) {
  const int32_t requested[AXIS_COUNT] = {radiansToVelocity(tilt), radiansToVelocity(pan)};
  if (requested[AXIS_Y] == 0 && requested[AXIS_X] == 0) {
    if (active) require(hold(), "velocity stop");
    return;
  }
  if (faulted || (active && !velocity_stream)) {
    readyForMove();
    return;
  }

  // Validate both axes before changing either motor.
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    int32_t cap;
    if (!require(readValue(ids[i], VELOCITY_LIMIT, cap) && cap > 0 &&
                 cap <= VELOCITY_LIMIT_MAX, "velocity limit")) return;
    if (abs(requested[i]) > cap) {
      require(hold(), "velocity limit stop");
      Serial.println("REJECTED: rad/s exceeds motor velocity limit.");
      return;
    }
  }

  const bool starting = !velocity_stream;
  if (starting) {
    if (!prepareAxis(AXIS_Y) || !prepareAxis(AXIS_X)) return;
    // Resample position before torque-on; an old boundary target must not resume.
    for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
      int32_t p;
      if (!require(readValue(ids[i], PRESENT_POSITION, p) && writePosition(ids[i], p),
                   "velocity initial hold")) return;
      targets[i] = p;
      if (!require(dxl.torqueOn(ids[i]), "torque on") ||
          !require(writeValue(ids[i], BUS_WATCHDOG, WATCHDOG_500MS), "watchdog")) return;
    }
  }

  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (!starting && requested[i] == stream_velocity[i]) continue;
    int32_t p;
    if (!require(readValue(ids[i], PRESENT_POSITION, p), "velocity position read")) return;
    const int32_t magnitude = abs(requested[i]);
    const int64_t goal = magnitude == 0 ? static_cast<int64_t>(p) :
        static_cast<int64_t>(anchors[i]) + (requested[i] > 0 ? limitTicks(i) : -limitTicks(i));
    if (!require(goal >= -EXTENDED_POSITION_MAX && goal <= EXTENDED_POSITION_MAX,
                 "velocity target range")) return;
    // Profile Velocity=0 means unlimited speed, not stop. Hold uses Goal Position.
    if (magnitude != 0 &&
        (!require(writeValue(ids[i], PROFILE_ACCELERATION, accelerationForProfile(magnitude)),
                  "velocity acceleration") ||
         !require(writeValue(ids[i], PROFILE_VELOCITY, magnitude), "velocity profile"))) return;
    if (!require(writePosition(ids[i], static_cast<int32_t>(goal)), "velocity goal")) return;
    targets[i] = static_cast<int32_t>(goal);
    stream_velocity[i] = requested[i];
  }
  velocity_command_ms = millis();
  velocity_stream = true;
  active = true;
  dual_move = false;
  sequence = false;
}

void updateVelocityStream() {
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    int32_t p;
    if (!require(readValue(ids[i], PRESENT_POSITION, p), "velocity feedback") ||
        !require(llabs(static_cast<int64_t>(p) - anchors[i]) <=
                 limitTicks(i) + ARRIVAL_TOLERANCE_TICKS, "velocity travel range")) return;
  }
}

// ============================================================
// 두 축 동시 이동 (DX DY SPEED)
// ============================================================

// Profile Acceleration(108), Profile Velocity(112), Goal Position(116)을
// 두 모터에 Sync Write 패킷 하나로 보내 함께 출발시킨다.
void moveBoth(const float values[4]) {
  if (!readyForMove()) return;

  // 입력 순서(X, Y)를 내부 순서(Y, X)로 바꾼다.
  const float deltas[AXIS_COUNT] = {values[1], values[0]};
  const float speeds[AXIS_COUNT] = {values[3], values[2]};
  int32_t profiles[AXIS_COUNT], goals[AXIS_COUNT], accelerations[AXIS_COUNT];

  // 1. 속도를 모터 단위로 바꾸고 각 모터의 상한과 비교
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    int32_t cap;
    if (!require(readValue(ids[i], VELOCITY_LIMIT, cap) && cap > 0 && cap <= VELOCITY_LIMIT_MAX,
                 "velocity limit")) return;
    profiles[i] = speedToProfile(speeds[i]);
    if (profiles[i] < 1 || profiles[i] > cap) {
      Serial.print("REJECTED: motor ID");
      Serial.print(ids[i]);
      Serial.print(" speed limit=");
      Serial.print(cap * VELOCITY_UNIT_DEG_S, 3);
      Serial.println(" deg/s.");
      return;
    }
  }

  // 2. 약 0.3초에 설정 속도에 도달하는 가속도. 짧은 이동에서는 도달 전에 감속한다.
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    accelerations[i] = accelerationForProfile(profiles[i]);
  }

  // 3. 축 준비
  if (!prepareAxis(AXIS_Y) || !prepareAxis(AXIS_X)) return;

  // 4. 목표 = 현재 목표 + 변화량. 홈 기준 범위를 벗어나면 거부한다.
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    const int64_t goal = static_cast<int64_t>(targets[i]) + degToTicks(deltas[i]);
    const int64_t rel = goal - anchors[i];
    if (llabs(rel) > limitTicks(i)) {
      Serial.print("REJECTED: ");
      Serial.print(i == AXIS_Y ? "Y" : "X");
      Serial.print(" result ");
      Serial.print(rel * 360.0 / TICKS_PER_REV, 2);
      Serial.print(" deg outside +/-");
      Serial.print(angle_limits[i], 0);
      Serial.print("; current ");
      Serial.print(ticksToDeg(relativeTarget(i)), 2);
      Serial.println(" deg from home.");
      return;
    }
    if (!require(goal >= -EXTENDED_POSITION_MAX && goal <= EXTENDED_POSITION_MAX, "extended position range")) return;
    goals[i] = static_cast<int32_t>(goal);
  }

  // 5. 토크 켜기. prepareAxis()가 현재 위치를 목표로 설정했으므로 아직 움직이지 않는다.
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (!require(dxl.torqueOn(ids[i]), "torque on") ||
        !require(writeValue(ids[i], BUS_WATCHDOG, WATCHDOG_500MS), "watchdog")) return;
  }

  // 6. 가속도·속도·목표 위치(12바이트)를 Sync Write로 동시 전송. 이 순간 두 축이 출발한다.
  struct Payload {
    int32_t acceleration, velocity, position;
  };
  static_assert(sizeof(Payload) == 12, "Sync Write payload must be 12 bytes");
  Payload data[AXIS_COUNT] = {
    {accelerations[AXIS_Y], profiles[AXIS_Y], goals[AXIS_Y]},
    {accelerations[AXIS_X], profiles[AXIS_X], goals[AXIS_X]},
  };

  DYNAMIXEL::XELInfoSyncWrite_t axes[AXIS_COUNT] = {};
  DYNAMIXEL::InfoSyncWriteInst_t packet = {};
  packet.addr = ADDR_PROFILE_ACCELERATION;
  packet.addr_length = sizeof(Payload);
  packet.p_xels = axes;
  packet.xel_count = AXIS_COUNT;
  packet.is_info_changed = true;
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    axes[i].id = ids[i];
    axes[i].p_data = reinterpret_cast<uint8_t *>(&data[i]);
  }
  if (!require(dxl.syncWrite(&packet), "dual target write")) return;

  // 7. Sync Write에는 응답이 없으므로 가속도·속도·목표 위치를 모두 읽어 확인한다.
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (!require(verifyValue(ids[i], PROFILE_ACCELERATION, accelerations[i]),
                 "dual acceleration verification") ||
        !require(verifyValue(ids[i], PROFILE_VELOCITY, profiles[i]),
                 "dual velocity verification") ||
        !require(verifyValue(ids[i], GOAL_POSITION, goals[i]),
                 "dual target verification")) return;
    targets[i] = goals[i];
    dual_speeds[i] = profiles[i] * VELOCITY_UNIT_DEG_S;
    dual_settled[i] = 0;
  }

  // 8. 상태 기록과 출력. 도착 판정은 loop()가 한다.
  stage_ms = millis();
  sequence = false;
  dual_move = true;
  active = true;

  Serial.print("MOVING XY: dX=");
  Serial.print(values[0], 2);
  Serial.print(" dY=");
  Serial.print(values[1], 2);
  Serial.print(" -> ");
  printXY(goals[AXIS_X] - anchors[AXIS_X], goals[AXIS_Y] - anchors[AXIS_Y]);
  Serial.print(" deg from home; actual profile X=");
  Serial.print(dual_speeds[AXIS_X], 3);
  Serial.print(" Y=");
  Serial.print(dual_speeds[AXIS_Y], 3);
  Serial.print(" deg/s; acceleration_raw X=");
  Serial.print(accelerations[AXIS_X]);
  Serial.print(" Y=");
  Serial.println(accelerations[AXIS_Y]);
}

// ============================================================
// 순차 이동 (홈 복귀·위치 복원)
// ============================================================

// Y축부터 rel 위치로 이동을 시작한다. Y축이 도착하면 loop()가 X축 이동을 이어서 한다.
void startSequence(const char *name, const int32_t rel[AXIS_COUNT]) {
  sequence_name = name;
  sequence_ticks[AXIS_Y] = rel[AXIS_Y];
  sequence_ticks[AXIS_X] = rel[AXIS_X];
  sequence = moveAxis(AXIS_Y, rel[AXIS_Y], RETURN_SPEED_DEG_S);
}

// h: 두 축을 준비하고 홈 위치로 순차 복귀한다.
void startHome() {
  if (!readyForMove()) return;
  if (!prepareAxis(AXIS_Y) || !prepareAxis(AXIS_X)) return;
  const int32_t home[AXIS_COUNT] = {0, 0};
  startSequence("HOME", home);
}

// 부팅 시 현재 위치를 저장 위치와 비교한다. 같으면 그대로 두고, 다르면 저장 위치로 순차 이동한다.
void startRestore() {
  if (!saved_valid) {
    Serial.println("RESTORE: no saved position; waiting for commands.");
    return;
  }

  // 모터 전원이 아직 없을 때 FAULT가 걸리지 않도록 먼저 ping으로 확인한다.
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (!dxl.ping(ids[i])) {
      Serial.print("RESTORE skipped: motor ID");
      Serial.print(ids[i]);
      Serial.println(" no response.");
      return;
    }
  }

  if (!prepareAxis(AXIS_Y) || !prepareAxis(AXIS_X)) return;

  bool same = true;
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    same = same && abs(relativeTarget(i) - saved_ticks[i]) <= RESTORE_TOLERANCE;
  }

  if (same) {
    Serial.print("POSITION OK: ");
  } else {
    Serial.print("POSITION CHANGED: now ");
    printXY(relativeTarget(AXIS_X), relativeTarget(AXIS_Y));
    Serial.print(" deg; ");
  }
  Serial.print("saved ");
  printXY(saved_ticks[AXIS_X], saved_ticks[AXIS_Y]);
  Serial.println(" deg from home.");

  if (!same) startSequence("RESTORE", saved_ticks);
}

// ============================================================
// 토크 해제와 진단
// ============================================================

// off: 이동을 취소하고 두 축의 토크와 감시 타이머를 끈다.
// 카메라 무게로 축이 내려갈 수 있으므로 먼저 받쳐야 한다.
void release() {
  active = false;
  sequence = false;
  dual_move = false;
  velocity_stream = false;
  stream_velocity[AXIS_Y] = stream_velocity[AXIS_X] = 0;

  bool ok = true;
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    bool axis_ok = dxl.torqueOff(ids[i]);
    if (axis_ok) axis_ok = writeValue(ids[i], BUS_WATCHDOG, 0);
    ok = axis_ok && ok;
    configured[i] = false;   // 다음 이동 때 홈 기준을 다시 계산한다.
  }
  Serial.println(ok ? "OFF: both axes released. Support camera." : "OFF failed: support camera and check power.");
}

// 모터 항목 하나를 읽어 "이름=값"으로 출력한다.
void scanField(uint8_t id, uint8_t item, const char *name) {
  int32_t value;
  const bool ok = readValue(id, item, value);
  Serial.print(name);
  Serial.print('=');
  if (ok) {
    Serial.println(value);
  } else {
    Serial.print("READ_ERROR lib=");
    Serial.print(dxl.getLastLibErrCode());
    Serial.print(" status=");
    Serial.println(dxl.getLastStatusPacketError());
  }
}

// scan: 별도 진단 스케치 없이 ID와 모터 상태를 확인한다. 설정은 바꾸지 않는다.
void scanMotors() {
  if (active) {
    Serial.println("BUSY: send x before scan.");
    return;
  }

  Serial.println("SCAN_BEGIN: IDs 11 and 12, Protocol 2, 1 Mbps");
  for (uint8_t id = 11; id <= 12; ++id) {
    if (!dxl.ping(id)) {
      Serial.print("NO_RESPONSE id=");
      Serial.println(id);
      continue;
    }
    Serial.print("FOUND id=");
    Serial.print(id);
    Serial.print(" model=");
    Serial.println(dxl.getModelNumber(id));
    scanField(id, FIRMWARE_VERSION, "firmware");
    scanField(id, DRIVE_MODE, "drive_mode");
    scanField(id, OPERATING_MODE, "operating_mode");
    scanField(id, BUS_WATCHDOG, "bus_watchdog_raw_20ms");
    scanField(id, PRESENT_POSITION, "position_ticks");
    scanField(id, PRESENT_VELOCITY, "velocity_raw");
    scanField(id, GOAL_POSITION, "goal_position_ticks");
    scanField(id, PROFILE_VELOCITY, "profile_velocity_raw");
    scanField(id, PROFILE_ACCELERATION, "profile_acceleration_raw");
    scanField(id, HOMING_OFFSET, "homing_offset");
    scanField(id, TORQUE_ENABLE, "torque_enable");
    scanField(id, HARDWARE_ERROR_STATUS, "hardware_error");
    scanField(id, PRESENT_INPUT_VOLTAGE, "input_voltage_raw_0.1V");
  }
  Serial.println("SCAN_DONE");
}

// ============================================================
// 명령 처리
// ============================================================

// x: 정지 후 현재 위치를 유지하고, 성공하면 그 위치를 저장한다.
void stopAndSave() {
  const bool ok = hold();
  Serial.println("STOP: holding current positions.");
  if (ok && !faulted) savePosition();
}

// auto on/off: 부팅 시 자동 홈 복귀 설정을 EEPROM에 저장한다.
// 확인용 번호와 반대 값(254/255)을 함께 저장해, 저장 공간이 깨져도 우연히 켜지지 않게 한다.
void saveAutoHome(bool enabled) {
  auto_home = enabled;
  BootSetting setting = {BOOT_MAGIC, static_cast<uint8_t>(auto_home), static_cast<uint8_t>(auto_home ? 254 : 255)};
  EEPROM.put(0, setting);

  BootSetting verify;
  EEPROM.get(0, verify);
  if (verify.magic != setting.magic || verify.enabled != setting.enabled || verify.inverse != setting.inverse) {
    auto_home = false;
    Serial.println("ERROR: boot setting verification failed.");
  } else {
    Serial.println(auto_home ? "AUTO on saved: home on next board startup." : "AUTO off saved: wait for commands.");
  }
}

// Raspberry Pi에서 받은 명령 한 줄을 알맞은 함수로 보낸다.
void command(char *line) {
  // 앞뒤 공백 제거
  while (isspace(static_cast<unsigned char>(*line))) ++line;
  size_t length = strlen(line);
  while (length && isspace(static_cast<unsigned char>(line[length - 1]))) line[--length] = '\0';
  if (!length) return;

  if (!strcmp(line, "h"))        { startHome();          return; }
  if (!strcmp(line, "x"))        { stopAndSave();        return; }
  if (!strcmp(line, "off"))      { release();            return; }
  if (!strcmp(line, "p"))        { printStatus();        return; }
  if (!strcmp(line, "scan"))     { scanMotors();         return; }
  if (!strcmp(line, "auto on"))  { saveAutoHome(true);   return; }
  if (!strcmp(line, "auto off")) { saveAutoHome(false);  return; }

  if (*line == 'v' && (!line[1] || isspace(static_cast<unsigned char>(line[1])))) {
    float pan, tilt;
    if (parseVelocity(line + 1, pan, tilt)) {
      commandVelocity(pan, tilt);
    } else {
      if (velocity_stream) require(hold(), "invalid velocity stop");
      Serial.println("REJECTED: use v PAN_RAD_S TILT_RAD_S.");
    }
    return;
  }

  float move_values[4];
  if (parseCommonMove(line, move_values)) {
    moveBoth(move_values);
    return;
  }

  // 숫자 명령 형식이 틀렸으면 입력 바이트를 16진수로 보여 준다.
  if (*line == '+' || *line == '-' || *line == '.' || isdigit(static_cast<unsigned char>(*line))) {
    Serial.print("INVALID XY: [");
    Serial.print(line);
    Serial.println("]");
    Serial.print("Input bytes:");
    for (size_t i = 0; i < length; ++i) {
      Serial.print(' ');
      Serial.print(static_cast<uint8_t>(line[i]), HEX);
    }
    Serial.println();
  }
  Serial.println("Use v PAN_RAD_S TILT_RAD_S | DX_DEG DY_DEG SPEED | h | x | off | p | scan | auto on/off.");
}

// ============================================================
// setup(): 전원을 켤 때 한 번
// ============================================================

void setup() {
  Serial.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);

  BootSetting setting;
  EEPROM.get(0, setting);
  auto_home = setting.magic == BOOT_MAGIC && setting.enabled == 1 && setting.inverse == 254;
  loadPosition();

  delay(2000);   // 모터 전원과 USB 연결이 안정될 시간

  // 입력 해석 기능 자체 점검
  float check[4];
  if (!parseCommonMove("10 10 10", check) ||
      check[0] != 10 || check[1] != 10 || check[2] != 10 || check[3] != 10) {
    faulted = true;
    reason = "XY parser self-check";
    Serial.println("FAULT: XY parser self-check failed.");
    return;
  }

  Serial.print(CONTROL_BUILD_ID);
  Serial.println(": v PAN_RAD_S TILT_RAD_S; legacy DX_DEG DY_DEG SPEED supported.");
  Serial.println("OPENCR_READY: bounded rad/s velocity; home range pan +/-180, tilt +/-135.");

  // 이동은 시작만 한다. 도착은 loop()가 기다리므로 복원 중에도 x로 멈출 수 있다.
  if (auto_home) {
    startHome();
  } else {
    startRestore();
  }
}

// ============================================================
// loop(): 계속 반복
// ============================================================

// 시리얼 입력을 한 줄씩 모아 command()로 넘긴다. x는 줄바꿈 없이 바로 정지한다.
void readSerialInput() {
  static char line[80];
  static uint8_t used = 0;
  static bool overflow = false;

  // 입력이 많아도 모터 점검이 밀리지 않게 한 번에 최대 32글자만 읽는다.
  for (uint8_t n = 0; n < 32 && Serial.available(); ++n) {
    const char c = Serial.read();

    if (c == 'x') {   // 비상 정지
      stopAndSave();
      used = 0;
      overflow = false;
      continue;
    }

    if (c == '\r' || c == '\n') {
      line[used] = '\0';
      if (overflow) {
        Serial.println("Command too long; discarded.");
      } else {
        command(line);
      }
      used = 0;
      overflow = false;
    } else if (c == '\b' || c == 127) {   // 백스페이스
      if (used && !overflow) --used;
    } else if (!overflow) {
      if (used < sizeof(line) - 1) {
        line[used++] = c;
      } else {
        overflow = true;
      }
    }
  }
}

// 준비된 축의 하드웨어 오류를 읽는다. 이 주기적인 통신이 모터 감시 타이머도 초기화한다.
bool checkAxesHealthy() {
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (!configured[i] || faulted) continue;
    int32_t error, watchdog;
    if (!require(readValue(ids[i], HARDWARE_ERROR_STATUS, error) && error == 0,
                 "communication or hardware error")) return false;
    if (!require(readValue(ids[i], BUS_WATCHDOG, watchdog), "watchdog read") ||
        !require(watchdog >= 0, "bus watchdog expired")) return false;
  }
  return true;
}

// 두 축 동시 이동의 도착 판정. 두 축이 각각 5번 연속 도착하면 완료한다.
void updateDualMove() {
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    int32_t p, v;
    if (!require(readValue(ids[i], PRESENT_POSITION, p) &&
                 readValue(ids[i], PRESENT_VELOCITY, v), "dual feedback")) return;

    if (isAtTarget(i, p, v)) {
      if (dual_settled[i] < SETTLE_COUNT) ++dual_settled[i];
    } else {
      dual_settled[i] = 0;
    }

    const uint32_t timeout_ms = moveTimeoutMs(i, dual_speeds[i]);
    if (dual_settled[i] < SETTLE_COUNT && !require(millis() - stage_ms < timeout_ms, "dual movement timeout")) return;
  }

  if (dual_settled[AXIS_Y] >= SETTLE_COUNT && dual_settled[AXIS_X] >= SETTLE_COUNT) {
    active = false;
    dual_move = false;
    Serial.println("DONE XY: both axes reached; holding position.");
    savePosition();
  }
}

// 순차 이동의 도착 판정. Y축이 도착하면 X축을 출발시키고, X축까지 도착하면 완료한다.
void updateSequenceMove() {
  const float actual_speed = speedToProfile(stage_speed) * VELOCITY_UNIT_DEG_S;
  if (!require(millis() - stage_ms < moveTimeoutMs(stage, actual_speed), "movement timeout")) return;

  int32_t p, v;
  if (!require(readValue(ids[stage], PRESENT_POSITION, p) && readValue(ids[stage], PRESENT_VELOCITY, v), "feedback")) return;

  settled = isAtTarget(stage, p, v) ? settled + 1 : 0;
  if (settled < SETTLE_COUNT) return;

  Serial.print("REACHED ");
  Serial.println(stage == AXIS_Y ? "tilt" : "pan");
  active = false;

  if (sequence && stage == AXIS_Y) {
    if (!moveAxis(AXIS_X, sequence_ticks[AXIS_X], stage_speed)) sequence = false;
  } else {
    // 도착 후에도 토크는 끄지 않는다. 카메라가 무게로 처지지 않게 위치를 유지한다.
    sequence = false;
    Serial.print("DONE ");
    Serial.print(sequence_name);
    Serial.println(": holding position; no automatic torque off.");
    savePosition();
  }
}

void loop() {
  // Status reads keep the bus watchdog alive, so velocity commands need their
  // own deadline. Check before input processing; stale commands must not persist.
  if (velocity_stream && millis() - velocity_command_ms >= VELOCITY_COMMAND_TIMEOUT_MS) {
    require(hold(), "velocity command timeout stop");
    Serial.println("STOP: velocity command timeout.");
  }
  // 1단계: 입력 처리 (매번)
  readSerialInput();

  // 2단계: 100ms마다 오류 점검
  if (millis() - poll_ms < POLL_INTERVAL_MS) return;
  poll_ms = millis();
  if (!checkAxesHealthy()) return;

  // 3단계: 도착 판정
  if (!active) return;
  if (velocity_stream) {
    updateVelocityStream();
  } else if (dual_move) {
    updateDualMove();
  } else {
    updateSequenceMove();
  }
}
