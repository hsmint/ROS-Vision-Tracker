// Camera controller: pan ID11 home4067, tilt ID12 home2116; homing offsets=0.
// Home-relative angles; motor's internal position controller holds after arrival.
// h: tilt then pan home. pan DEG / tilt DEG. speed DEG_S. x: hold. off: release.
// move X_DEG Y_DEG X_SPEED Y_SPEED: simultaneous home-relative movement.
// auto on/off: persist boot homing preference. Default off for first motion test.
#include <Dynamixel2Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
using namespace ControlTableItem;
Dynamixel2Arduino dxl(Serial3, 84);
const uint8_t ids[] = {12, 11};
const int32_t homes[] = {2116, 4067};
const float angle_limits[] = {135, 90};
int32_t targets[2], anchors[2];
bool configured[2] = {false, false};
bool active = false, home_sequence = false, faulted = false;
bool dual_move = false;
uint8_t dual_settled[2] = {0, 0};
float dual_speeds[2] = {5, 5};
bool auto_home = false;
float speed_deg_s = 5;
uint8_t stage = 0, settled = 0;
uint32_t stage_ms = 0, poll_ms = 0;
const char *reason = "none";
struct BootSetting { uint32_t magic; uint8_t enabled; uint8_t inverse; };
const uint32_t BOOT_MAGIC = 0x43414D32;

// 메모: 지정한 모터의 제어 테이블 항목을 읽어 value에 넣는다. 통신·상태 오류가 없을 때만 true를 반환한다.
bool readValue(uint8_t id, uint8_t item, int32_t &value) {
  value = dxl.readControlTableItem(item, id, 30);
  return dxl.getLastLibErrCode() == DXL_LIB_OK && dxl.getLastStatusPacketError() == 0;
}
// 메모: 4096틱 한 바퀴 경계를 고려해 현재 위치에서 홈까지의 가장 가까운 부호 있는 차이를 구한다. 케이블의 누적 감김은 판별하지 못한다.
int32_t nearestError(int32_t position, int32_t home) {
  int32_t e = (home - position % 4096) % 4096;
  if (e > 2048) e -= 4096;
  if (e < -2048) e += 4096;
  return e;
}
// 메모: 진행 중 이동·홈 복귀를 취소하고, 제어 중인 축의 현재 위치를 새 목표로 잡아 토크를 유지한다. 위치 유지 명령이 실패하면 토크 해제를 요청한다.
void hold() {
  active = false; home_sequence = false; dual_move = false;
  for (uint8_t i = 0; i < 2; ++i) {
    if (!configured[i]) continue;
    int32_t p, torque;
    if (!readValue(ids[i], TORQUE_ENABLE, torque)) {
      Serial.println("WARNING: torque status unreadable; support camera."); continue;
    }
    if (!torque) continue;
    if (!readValue(ids[i], PRESENT_POSITION, p) || !dxl.setGoalPosition(ids[i], p, UNIT_RAW)) {
      dxl.torqueOff(ids[i]);
      Serial.println("WARNING: hold failed; torque off requested; support camera.");
    }
  }
}
// 메모: 작업 성공 여부를 검사한다. 실패하면 원인을 저장하고 FAULT를 유지하며 hold()로 정지를 시도한다.
bool require(bool ok, const char *why) {
  if (!ok) {
    reason = why; faulted = true; hold();
    Serial.print("FAULT: "); Serial.println(reason);
  }
  return ok;
}
// 메모: 제어 상태, 속도, 자동 복귀 설정과 두 축의 엔코더·홈 기준 각도·토크·오류를 출력한다. 읽기 실패는 READ_ERROR로 표시한다.
void printStatus() {
  Serial.print("STATE="); Serial.print(faulted ? "FAULT" : active ? "MOVING" : "IDLE");
  Serial.print(" reason="); Serial.print(reason);
  Serial.print(" speed_deg_s="); Serial.print(speed_deg_s);
  Serial.print(" auto_home="); Serial.println(auto_home ? "on" : "off");
  for (uint8_t i = 0; i < 2; ++i) {
    int32_t p, torque, error;
    Serial.print(i == 0 ? "tilt ID12 " : "pan ID11 ");
    if (!readValue(ids[i], PRESENT_POSITION, p) ||
        !readValue(ids[i], TORQUE_ENABLE, torque) ||
        !readValue(ids[i], HARDWARE_ERROR_STATUS, error)) { Serial.println("READ_ERROR"); continue; }
    Serial.print("ticks="); Serial.print(p);
    Serial.print(" home_deg=");
    Serial.print((configured[i] ? static_cast<double>(p) - anchors[i] : -nearestError(p, homes[i])) * 360.0 / 4096.0, 2);
    Serial.print(" torque="); Serial.print(torque);
    Serial.print(" hardware_error="); Serial.println(error);
  }
}
// 메모: 축 번호 i(0=상하12, 1=좌우11)의 모델·오류·설정을 검사하고 Extended Position 모드를 준비한다. 처음 활성화할 때 홈 기준을 정하고 현재 위치를 초기 목표로 설정한다.
bool prepareAxis(uint8_t i) {
  const uint8_t id = ids[i];
  if (!require(dxl.ping(id), "ping") ||
      !require(dxl.getModelNumber(id) == XM430_W350, "model")) return false;
  int32_t offset, error, p, mode, drive;
  if (!require(readValue(id, HOMING_OFFSET, offset) && offset == 0, "home offset changed") ||
      !require(readValue(id, HARDWARE_ERROR_STATUS, error) && error == 0, "hardware error") ||
      !require(readValue(id, DRIVE_MODE, drive) && (drive & 4) == 0, "requires velocity profile") ||
      !require(readValue(id, OPERATING_MODE, mode), "mode read")) return false;
  if (mode != OP_EXTENDED_POSITION) {
    if (!require(dxl.torqueOff(id), "torque off") ||
        !require(dxl.setOperatingMode(id, OP_EXTENDED_POSITION), "extended position mode")) return false;
    configured[i] = false;
  }
  if (!require(readValue(id, PRESENT_POSITION, p), "position read")) return false;
  if (!configured[i]) {
    const int32_t e = nearestError(p, homes[i]);
    if (abs(e) > angle_limits[i] * 4096.0f / 360.0f) {
      Serial.println("REJECTED: current position outside home range; support and reposition with off.");
      return false;
    }
    anchors[i] = p + e;
  }
  if (!require(dxl.writeControlTableItem(BUS_WATCHDOG, id, 0), "watchdog clear") ||
      !require(dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 1), "acceleration") ||
      !require(dxl.setGoalPosition(id, p, UNIT_RAW), "initial hold target")) return false;
  configured[i] = true;
  return true;
}
// 메모: 홈 기준 angle 각도로 지정 축을 이동시킨다. 속도를 모터 단위로 변환하고 한계를 검사한 뒤 토크·watchdog·목표 위치를 설정한다. 도착 판정은 loop()가 수행한다.
bool moveAxis(uint8_t i, float angle) {
  int32_t p, cap;
  const uint8_t id = ids[i];
  if (!require(readValue(id, PRESENT_POSITION, p), "position read") ||
      !require(readValue(id, VELOCITY_LIMIT, cap) && cap > 0 && cap <= 1023, "velocity limit")) return false;
  const int32_t profile = static_cast<int32_t>(floorf(speed_deg_s / (0.229f * 6)));
  if (profile < 1 || profile > cap) { Serial.println("REJECTED: speed outside motor limit."); return false; }
  const int64_t target = static_cast<int64_t>(anchors[i]) + lroundf(angle * 4096.0f / 360.0f);
  if (!require(target >= -1048575 && target <= 1048575, "extended position range")) return false;
  if (!require(dxl.writeControlTableItem(PROFILE_VELOCITY, id, profile), "profile velocity") ||
      !require(dxl.setGoalPosition(id, p, UNIT_RAW), "stage hold") ||
      !require(dxl.torqueOn(id), "torque on") ||
      !require(dxl.writeControlTableItem(BUS_WATCHDOG, id, 25), "watchdog") ||
      !require(dxl.setGoalPosition(id, target, UNIT_RAW), "target write")) return false;
  targets[i] = target;
  stage = i; stage_ms = millis(); settled = 0; active = true;
  Serial.print("MOVING "); Serial.print(i == 0 ? "tilt " : "pan ");
  Serial.print(angle, 2); Serial.println(" deg relative to home.");
  return true;
}
// 메모: FAULT 또는 이동 중이면 새 이동 요청을 거부한다. 움직이는 동안에는 x로 정지한 뒤 새 명령을 보내야 한다.
bool readyForMove() {
  if (faulted) { Serial.print("FAULT: "); Serial.println(reason); return false; }
  if (active) { Serial.println("BUSY: send x to stop first."); return false; }
  return true;
}
// 입력 순서는 X각도 Y각도 X속도 Y속도. 내부 배열은 Y(ID12), X(ID11).
// 전체 입력을 검사한 다음에만 호출하므로 잘못된 입력으로 한 축만 이동하지 않는다.
bool parseMove(const char *text, float values[4]) {
  float parsed[4];
  for (uint8_t i = 0; i < 4; ++i) {
    while (isspace(static_cast<unsigned char>(*text))) ++text;
    char *end;
    parsed[i] = strtof(text, &end);
    if (end == text || !isfinite(parsed[i]) ||
        (*end && !isspace(static_cast<unsigned char>(*end)))) return false;
    text = end;
  }
  while (isspace(static_cast<unsigned char>(*text))) ++text;
  if (*text || fabsf(parsed[0]) > angle_limits[1] || fabsf(parsed[1]) > angle_limits[0] ||
      parsed[2] < 1.374f || parsed[2] > 30 || parsed[3] < 1.374f || parsed[3] > 30) return false;
  memcpy(values, parsed, sizeof(parsed));
  return true;
}

// Profile Acceleration(108), Profile Velocity(112), Goal Position(116)를
// 두 모터에 하나의 Sync Write 패킷으로 보내 함께 출발시킨다.
void moveBoth(const float values[4]) {
  if (!readyForMove()) return;
  const float angles[2] = {values[1], values[0]};
  const float speeds[2] = {values[3], values[2]};
  int32_t profiles[2], goals[2];
  for (uint8_t i = 0; i < 2; ++i) {
    int32_t cap;
    if (!require(readValue(ids[i], VELOCITY_LIMIT, cap) && cap > 0 && cap <= 1023,
                 "velocity limit")) return;
    profiles[i] = static_cast<int32_t>(floorf(speeds[i] / 1.374f));
    if (profiles[i] < 1 || profiles[i] > cap) {
      Serial.println("REJECTED: speed outside motor limit."); return;
    }
  }
  if (!prepareAxis(0) || !prepareAxis(1)) return;
  for (uint8_t i = 0; i < 2; ++i) {
    const int64_t goal = static_cast<int64_t>(anchors[i]) + lroundf(angles[i] * 4096.0f / 360.0f);
    if (!require(goal >= -1048575 && goal <= 1048575, "extended position range")) return;
    goals[i] = static_cast<int32_t>(goal);
  }
  // prepareAxis()가 현재 위치를 목표로 설정했으므로 토크를 켜도 새 이동은 아직 시작하지 않는다.
  for (uint8_t i = 0; i < 2; ++i) {
    if (!require(dxl.torqueOn(ids[i]), "torque on") ||
        !require(dxl.writeControlTableItem(BUS_WATCHDOG, ids[i], 25), "watchdog")) return;
  }
  struct Payload { int32_t acceleration, velocity, position; };
  static_assert(sizeof(Payload) == 12, "Sync Write payload must be 12 bytes");
  Payload data[2] = {{1, profiles[0], goals[0]}, {1, profiles[1], goals[1]}};
  DYNAMIXEL::XELInfoSyncWrite_t axes[2] = {};
  DYNAMIXEL::InfoSyncWriteInst_t packet = {};
  packet.addr = 108; packet.addr_length = sizeof(Payload);
  packet.p_xels = axes; packet.xel_count = 2; packet.is_info_changed = true;
  for (uint8_t i = 0; i < 2; ++i) {
    axes[i].id = ids[i]; axes[i].p_data = reinterpret_cast<uint8_t *>(&data[i]);
  }
  if (!require(dxl.syncWrite(&packet), "dual target write")) return;
  // Sync Write에는 개별 응답이 없으므로 각 모터의 적용된 목표를 확인한다.
  for (uint8_t i = 0; i < 2; ++i) {
    int32_t actual;
    if (!require(readValue(ids[i], GOAL_POSITION, actual) && actual == goals[i],
                 "dual target verification")) return;
    targets[i] = goals[i]; dual_speeds[i] = profiles[i] * 1.374f;
    dual_settled[i] = 0;
  }
  stage_ms = millis(); home_sequence = false; dual_move = true; active = true;
  Serial.print("MOVING XY: X="); Serial.print(values[0], 2);
  Serial.print(" Y="); Serial.print(values[1], 2);
  Serial.print(" deg; actual profile X="); Serial.print(dual_speeds[1], 3);
  Serial.print(" Y="); Serial.print(dual_speeds[0], 3); Serial.println(" deg/s.");
}

// 메모: 두 축을 준비하고 상하 축부터 홈 이동을 시작한다. 상하 도착 후 좌우 이동은 loop()에서 이어서 실행한다.
void startHome() {
  if (!readyForMove()) return;
  if (!prepareAxis(0) || !prepareAxis(1)) return;
  home_sequence = moveAxis(0, 0);
}
// 메모: 두 축의 이동을 취소하고 토크를 끈 뒤 watchdog 해제를 요청한다. 무게로 카메라가 내려갈 수 있으므로 먼저 지지해야 한다.
void release() {
  active = false; home_sequence = false; dual_move = false;
  bool ok = true;
  for (uint8_t i = 0; i < 2; ++i) {
    bool axis_ok = dxl.torqueOff(ids[i]);
    if (axis_ok) axis_ok = dxl.writeControlTableItem(BUS_WATCHDOG, ids[i], 0);
    ok = axis_ok && ok;
    configured[i] = false;
  }
  Serial.println(ok ? "OFF: both axes released. Support camera." : "OFF failed: support camera and check power.");
}
// 메모: 문자열 전체가 유한한 숫자인지 검사하고 value에 담는다. NaN·Inf·쉼표·불필요한 후행 문자열은 거부한다.
bool parseNumber(const char *text, float &value) {
  char *end;
  value = strtof(text, &end);
  if (end == text || !isfinite(value)) return false;
  while (isspace(static_cast<unsigned char>(*end))) ++end;
  return *end == '\0';
}
// 메모: 한 줄 명령을 해석한다. pan/tilt는 홈 기준 각도, speed는 이후 이동 속도이며 h/x/off/p와 EEPROM에 저장하는 auto on/off도 처리한다.
void command(char *line) {
  while (isspace(static_cast<unsigned char>(*line))) ++line;
  size_t length = strlen(line);
  while (length && isspace(static_cast<unsigned char>(line[length - 1]))) line[--length] = '\0';
  if (!length) return;
  if (!strcmp(line, "h")) { startHome(); return; }
  if (!strcmp(line, "x")) { hold(); Serial.println("STOP: holding current positions."); return; }
  if (!strcmp(line, "off")) { release(); return; }
  if (!strcmp(line, "p")) { printStatus(); return; }
  if (!strcmp(line, "auto on") || !strcmp(line, "auto off")) {
    auto_home = !strcmp(line, "auto on");
    BootSetting setting = {BOOT_MAGIC, static_cast<uint8_t>(auto_home), static_cast<uint8_t>(auto_home ? 254 : 255)};
    EEPROM.put(0, setting);
    BootSetting verify; EEPROM.get(0, verify);
    if (verify.magic != setting.magic || verify.enabled != setting.enabled || verify.inverse != setting.inverse) {
      auto_home = false; Serial.println("ERROR: boot setting verification failed.");
    } else Serial.println(auto_home ? "AUTO on saved: home on next board startup." : "AUTO off saved: wait for commands.");
    return;
  }
  if (!strncmp(line, "move", 4) && (line[4] == '\0' || isspace(static_cast<unsigned char>(line[4])))) {
    float values[4];
    if (!parseMove(line + 4, values)) {
      Serial.println("Use move X_DEG Y_DEG X_SPEED Y_SPEED. X +/-90, Y +/-135; speeds 1.374..30 deg/s."); return;
    }
    moveBoth(values); return;
  }
  float value;
  if (!strncmp(line, "speed ", 6) && parseNumber(line + 6, value)) {
    if (value < 1.374f || value > 30) { Serial.println("REJECTED: speed 1.374..30 deg/s."); return; }
    if (active) { Serial.println("BUSY: send x first."); return; }
    speed_deg_s = value; Serial.print("SPEED deg/s="); Serial.println(value); return;
  }
  const bool pan = !strncmp(line, "pan ", 4);
  const bool tilt = !strncmp(line, "tilt ", 5);
  if ((pan || tilt) && parseNumber(line + (pan ? 4 : 5), value)) {
    uint8_t i = pan ? 1 : 0;
    if (fabsf(value) > angle_limits[i]) { Serial.println("REJECTED: pan -90..90, tilt -135..135."); return; }
    if (!readyForMove() || !prepareAxis(i)) return;
    home_sequence = false; moveAxis(i, value); return;
  }
  Serial.println("Use move X_DEG Y_DEG X_SPEED Y_SPEED | pan DEG | tilt DEG | speed DEG_S | h | x | off | p | auto on/off.");
}
// 메모: 보드 시작 시 시리얼·모터 통신을 준비하고 EEPROM의 자동 복귀 설정을 읽는다. auto가 켜져 있으면 홈 복귀를 시작하고, 꺼져 있으면 입력을 기다린다.
void setup() {
  Serial.begin(115200);
  dxl.begin(1000000); dxl.setPortProtocolVersion(2.0);
  BootSetting setting; EEPROM.get(0, setting);
  auto_home = setting.magic == BOOT_MAGIC && setting.enabled == 1 && setting.inverse == 254;
  delay(2000);
  Serial.println("CAMERA_READY: home-relative angles. pan +/-90, tilt +/-135; speed 5 deg/s.");
  if (auto_home) startHome();
}
// 메모: 시리얼 입력과 즉시 정지 x를 처리하고 100ms마다 오류·위치·속도를 확인한다. 도착하면 다음 홈 축을 실행하거나 토크를 유지하며 이동을 완료한다.
void loop() {
  static char line[80]; static uint8_t used = 0; static bool overflow = false;
  for (uint8_t n = 0; n < 32 && Serial.available(); ++n) {
    char c = Serial.read();
    if (c == 'x') { hold(); used = 0; overflow = false; Serial.println("STOP: holding current positions."); continue; }
    if (c == '\r' || c == '\n') {
      line[used] = '\0';
      if (overflow) Serial.println("Command too long; discarded."); else command(line);
      used = 0; overflow = false;
    } else if (c == '\b' || c == 127) { if (used && !overflow) --used; }
    else if (!overflow) { if (used < sizeof(line) - 1) line[used++] = c; else overflow = true; }
  }
  if (millis() - poll_ms < 100) return;
  poll_ms = millis();
  for (uint8_t i = 0; i < 2; ++i) {
    if (!configured[i] || faulted) continue;
    int32_t error;
    if (!require(readValue(ids[i], HARDWARE_ERROR_STATUS, error) && error == 0,
                 "communication or hardware error")) return;
  }
  if (!active) return;
  if (dual_move) {
    for (uint8_t i = 0; i < 2; ++i) {
      const uint32_t timeout = 15000 + static_cast<uint32_t>(ceilf(1000 * 2 * angle_limits[i] / dual_speeds[i]));
      int32_t p, v;
      if (!require(readValue(ids[i], PRESENT_POSITION, p) &&
                   readValue(ids[i], PRESENT_VELOCITY, v), "dual feedback")) return;
      const bool reached = llabs(static_cast<int64_t>(p) - targets[i]) <= 8 && abs(v) <= 1;
      dual_settled[i] = reached ? (dual_settled[i] < 5 ? dual_settled[i] + 1 : 5) : 0;
      if (dual_settled[i] < 5 && !require(millis() - stage_ms < timeout, "dual movement timeout")) return;
    }
    if (dual_settled[0] >= 5 && dual_settled[1] >= 5) {
      active = false; dual_move = false;
      Serial.println("DONE XY: both axes reached; holding position.");
    }
    return;
  }
  const uint32_t timeout_ms = 15000 + static_cast<uint32_t>(ceilf(1000 * 270 / (floorf(speed_deg_s / 1.374f) * 1.374f)));
  if (!require(millis() - stage_ms < timeout_ms, "movement timeout")) return;
  int32_t p, v;
  if (!require(readValue(ids[stage], PRESENT_POSITION, p) && readValue(ids[stage], PRESENT_VELOCITY, v), "feedback")) return;
  settled = (llabs(static_cast<int64_t>(p) - targets[stage]) <= 8 && abs(v) <= 1) ? settled + 1 : 0;
  if (settled < 5) return;
  Serial.print("REACHED "); Serial.println(stage == 0 ? "tilt" : "pan");
  active = false;
  if (home_sequence && stage == 0) { if (!moveAxis(1, 0)) home_sequence = false; }
  else { home_sequence = false; Serial.println("DONE: holding position; no automatic torque off."); }
}
