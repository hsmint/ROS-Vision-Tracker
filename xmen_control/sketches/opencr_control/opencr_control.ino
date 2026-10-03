// OpenCR 통합 제어: X ID11 home0, Y ID12 home2048, 1 Mbps.
// Raspberry Pi main.py -> USB Serial -> OpenCR -> DYNAMIXEL.
// DX_DEG DY_DEG SPEED | h | x | off | p | scan | auto on/off.
// Moves are relative to the current target; results must stay within home +/-limits:
// X +/-180, Y +/-135. Last stopped position is saved and restored on startup.
#include <Dynamixel2Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
using namespace ControlTableItem;
Dynamixel2Arduino dxl(Serial3, 84);
const char CONTROL_BUILD_ID[] = "OPENCR_CONTROL_V2";
const uint8_t ids[] = {12, 11};
const int32_t homes[] = {2048, 0};  // Y=180 deg, X=0 deg; internal order Y, X
const float angle_limits[] = {135, 180};  // Y +/-135, X +/-180 deg
int32_t targets[2], anchors[2];
bool configured[2] = {false, false};
bool active = false, sequence = false, faulted = false;
bool dual_move = false;
uint8_t dual_settled[2] = {0, 0};
float dual_speeds[2] = {5, 5};
bool auto_home = false;
const float RETURN_SPEED_DEG_S = 30;  // 홈 복귀와 시작 시 위치 복원 속도
float stage_speed = RETURN_SPEED_DEG_S;
const char *sequence_name = "HOME";
int32_t sequence_ticks[2] = {0, 0};  // 순차 이동 목표, 홈 기준 틱; internal order Y, X
uint8_t stage = 0, settled = 0;
uint32_t stage_ms = 0, poll_ms = 0;
const char *reason = "none";
struct BootSetting { uint32_t magic; uint8_t enabled; uint8_t inverse; };
const uint32_t BOOT_MAGIC = 0x43414D32;
// 마지막 정지 위치(홈 기준 틱). BootSetting(0..7) 뒤 주소 16에 저장한다.
struct SavedPosition { uint32_t magic; int32_t ticks[2]; uint32_t check; };
const int POSITION_ADDR = 16;
const uint32_t POSITION_MAGIC = 0x504F5331;
const int32_t RESTORE_TOLERANCE = 12;  // 약 1.05 deg 이내면 같은 위치로 본다.
bool saved_valid = false;
int32_t saved_ticks[2] = {0, 0};

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
int32_t limitTicks(uint8_t i) { return lroundf(angle_limits[i] * 4096.0f / 360.0f); }
float ticksToDeg(int32_t ticks) { return ticks * 360.0f / 4096.0f; }
uint32_t positionCheck(const SavedPosition &s) {
  return ~(s.magic ^ static_cast<uint32_t>(s.ticks[0]) * 2654435761u ^ static_cast<uint32_t>(s.ticks[1]));
}
// EEPROM의 마지막 위치를 읽는다. 손상됐거나 범위를 벗어나면 사용하지 않는다.
void loadPosition() {
  SavedPosition s; EEPROM.get(POSITION_ADDR, s);
  saved_valid = s.magic == POSITION_MAGIC && s.check == positionCheck(s) &&
                abs(s.ticks[0]) <= limitTicks(0) && abs(s.ticks[1]) <= limitTicks(1);
  if (saved_valid) { saved_ticks[0] = s.ticks[0]; saved_ticks[1] = s.ticks[1]; }
}
// 메모: 진행 중 이동·홈 복귀를 취소하고, 제어 중인 축의 현재 위치를 새 목표로 잡아 토크를 유지한다. 위치 유지 명령이 실패하면 토크 해제를 요청한다.
bool hold() {
  active = false; sequence = false; dual_move = false;
  bool ok = true;
  for (uint8_t i = 0; i < 2; ++i) {
    if (!configured[i]) continue;
    int32_t p, torque;
    if (!readValue(ids[i], TORQUE_ENABLE, torque)) {
      Serial.println("WARNING: torque status unreadable; support camera."); ok = false; continue;
    }
    if (!torque) continue;
    if (!readValue(ids[i], PRESENT_POSITION, p) || !dxl.setGoalPosition(ids[i], p, UNIT_RAW)) {
      dxl.torqueOff(ids[i]); ok = false;
      Serial.println("WARNING: hold failed; torque off requested; support camera.");
    } else targets[i] = p;
  }
  return ok;
}
// 두 축의 현재 목표를 홈 기준 틱으로 EEPROM에 저장한다. 값이 같으면 쓰지 않는다.
void savePosition() {
  if (!configured[0] || !configured[1]) return;
  SavedPosition s = {POSITION_MAGIC, {targets[0] - anchors[0], targets[1] - anchors[1]}, 0};
  s.check = positionCheck(s);
  if (saved_valid && s.ticks[0] == saved_ticks[0] && s.ticks[1] == saved_ticks[1]) return;
  EEPROM.put(POSITION_ADDR, s);
  SavedPosition verify; EEPROM.get(POSITION_ADDR, verify);
  if (memcmp(&verify, &s, sizeof(s))) { Serial.println("WARNING: position save failed."); return; }
  saved_valid = true; saved_ticks[0] = s.ticks[0]; saved_ticks[1] = s.ticks[1];
  Serial.print("SAVED: X="); Serial.print(ticksToDeg(s.ticks[1]), 2);
  Serial.print(" Y="); Serial.print(ticksToDeg(s.ticks[0]), 2); Serial.println(" deg from home.");
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
  Serial.print("FIRMWARE="); Serial.println(CONTROL_BUILD_ID);
  Serial.print("STATE="); Serial.print(faulted ? "FAULT" : active ? "MOVING" : "IDLE");
  Serial.print(" reason="); Serial.print(reason);
  Serial.print(" return_speed_deg_s="); Serial.print(RETURN_SPEED_DEG_S);
  Serial.print(" auto_home="); Serial.println(auto_home ? "on" : "off");
  Serial.print("SAVED_POSITION=");
  if (saved_valid) {
    Serial.print("X="); Serial.print(ticksToDeg(saved_ticks[1]), 2);
    Serial.print(" Y="); Serial.print(ticksToDeg(saved_ticks[0]), 2); Serial.println(" deg");
  } else Serial.println("none");
  for (uint8_t i = 0; i < 2; ++i) {
    int32_t p, torque, error, cap;
    Serial.print(i == 0 ? "tilt ID12 " : "pan ID11 ");
    if (!readValue(ids[i], PRESENT_POSITION, p) ||
        !readValue(ids[i], TORQUE_ENABLE, torque) ||
        !readValue(ids[i], HARDWARE_ERROR_STATUS, error) ||
        !readValue(ids[i], VELOCITY_LIMIT, cap)) { Serial.println("READ_ERROR"); continue; }
    Serial.print("ticks="); Serial.print(p);
    Serial.print(" home_deg=");
    Serial.print((configured[i] ? static_cast<double>(p) - anchors[i] : -nearestError(p, homes[i])) * 360.0 / 4096.0, 2);
    Serial.print(" torque="); Serial.print(torque);
    Serial.print(" hardware_error="); Serial.print(error);
    Serial.print(" velocity_limit_deg_s="); Serial.println(cap * 1.374f, 3);
  }
}
// 메모: 축 번호 i(0=상하12, 1=좌우11)의 모델·오류·설정을 검사하고 Extended Position 모드를 준비한다. 처음 활성화할 때 홈 기준을 정하고 현재 위치를 초기 목표로 설정한다.
bool prepareAxis(uint8_t i) {
  const uint8_t id = ids[i];
  if (!require(dxl.ping(id), "ping") ||
      !require(dxl.getModelNumber(id) == XM430_W350, "model")) return false;
  int32_t offset, error, p, mode, drive, torque;
  if (!require(readValue(id, TORQUE_ENABLE, torque), "torque read")) return false;
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
  // 토크를 유지 중이면 마지막 목표를 상대 이동의 기준으로 이어 쓴다. 아니면 현재 위치가 기준이다.
  const bool keep_target = configured[i] && torque;
  if (!configured[i]) {
    // 가장 가까운 홈 기준을 고르되, 저장 위치가 있으면 그 위치와 가장 가까운 바퀴를 고른다.
    int32_t rel = -nearestError(p, homes[i]);
    if (saved_valid) rel += 4096 * lroundf((saved_ticks[i] - rel) / 4096.0f);
    if (abs(rel) > limitTicks(i)) {
      Serial.println("REJECTED: current position outside home range; support and reposition with off.");
      return false;
    }
    anchors[i] = p - rel;
  }
  if (!keep_target) targets[i] = p;
  if (!require(dxl.writeControlTableItem(BUS_WATCHDOG, id, 0), "watchdog clear") ||
      !require(dxl.writeControlTableItem(PROFILE_ACCELERATION, id, 1), "acceleration") ||
      !require(dxl.setGoalPosition(id, targets[i], UNIT_RAW), "initial hold target")) return false;
  configured[i] = true;
  return true;
}
// 메모: 홈 기준 rel 틱 위치로 지정 축을 speed(deg/s)로 이동시킨다. 속도를 모터 단위로 변환하고 한계를 검사한 뒤 토크·watchdog·목표 위치를 설정한다. 도착 판정은 loop()가 수행한다.
bool moveAxis(uint8_t i, int32_t rel, float speed) {
  int32_t p, cap;
  const uint8_t id = ids[i];
  if (!require(readValue(id, PRESENT_POSITION, p), "position read") ||
      !require(readValue(id, VELOCITY_LIMIT, cap) && cap > 0 && cap <= 1023, "velocity limit")) return false;
  const int32_t profile = static_cast<int32_t>(floorf(speed / 1.374f));
  if (profile < 1 || profile > cap) { Serial.println("REJECTED: speed outside motor limit."); return false; }
  const int64_t target = static_cast<int64_t>(anchors[i]) + rel;
  if (!require(target >= -1048575 && target <= 1048575, "extended position range")) return false;
  if (!require(dxl.writeControlTableItem(PROFILE_VELOCITY, id, profile), "profile velocity") ||
      !require(dxl.setGoalPosition(id, p, UNIT_RAW), "stage hold") ||
      !require(dxl.torqueOn(id), "torque on") ||
      !require(dxl.writeControlTableItem(BUS_WATCHDOG, id, 25), "watchdog") ||
      !require(dxl.setGoalPosition(id, target, UNIT_RAW), "target write")) return false;
  targets[i] = target;
  stage = i; stage_ms = millis(); settled = 0; active = true; stage_speed = speed;
  Serial.print("MOVING "); Serial.print(i == 0 ? "tilt " : "pan ");
  Serial.print(ticksToDeg(rel), 2); Serial.print(" deg from home at ");
  Serial.print(profile * 1.374f, 2); Serial.println(" deg/s.");
  return true;
}
// 메모: FAULT 또는 이동 중이면 새 이동 요청을 거부한다. 움직이는 동안에는 x로 정지한 뒤 새 명령을 보내야 한다.
bool readyForMove() {
  if (faulted) { Serial.print("FAULT: "); Serial.println(reason); return false; }
  if (active) { Serial.println("BUSY: send x to stop first."); return false; }
  return true;
}
// 시리얼 입력의 부호 있는 십진수를 직접 읽는다. 동적 메모리 할당을 사용하지 않는다.
bool readDecimal(const char *&text, float &value) {
  while (isspace(static_cast<unsigned char>(*text))) ++text;
  bool negative = false;
  if (*text == '+' || *text == '-') { negative = *text == '-'; ++text; }
  bool digit = false;
  double number = 0;
  while (*text >= '0' && *text <= '9') {
    digit = true; number = number * 10 + (*text++ - '0');
    if (number > 1000000) return false;
  }
  if (*text == '.') {
    ++text; double place = 0.1;
    while (*text >= '0' && *text <= '9') {
      digit = true; number += (*text++ - '0') * place; place *= 0.1;
    }
  }
  if (!digit || (*text && !isspace(static_cast<unsigned char>(*text)))) return false;
  value = static_cast<float>(negative ? -number : number);
  return isfinite(value);
}

// X변화량 Y변화량 공통속도: 현재 위치 기준이며 속도를 두 축에 동일하게 적용한다.
// 최종 위치의 홈 기준 범위는 moveBoth()에서 검사한다.
bool parseCommonMove(const char *text, float values[4]) {
  float parsed[3];
  for (uint8_t i = 0; i < 3; ++i) {
    if (!readDecimal(text, parsed[i])) return false;
  }
  while (isspace(static_cast<unsigned char>(*text))) ++text;
  if (*text || fabsf(parsed[0]) > 2 * angle_limits[1] || fabsf(parsed[1]) > 2 * angle_limits[0] ||
      parsed[2] < 1.374f) return false;  // 상한은 이동 전에 각 모터의 Velocity Limit으로 검사한다.
  values[0] = parsed[0]; values[1] = parsed[1];
  values[2] = values[3] = parsed[2];
  return true;
}

// Profile Acceleration(108), Profile Velocity(112), Goal Position(116)를
// 두 모터에 하나의 Sync Write 패킷으로 보내 함께 출발시킨다.
void moveBoth(const float values[4]) {
  if (!readyForMove()) return;
  const float deltas[2] = {values[1], values[0]};
  const float speeds[2] = {values[3], values[2]};
  int32_t profiles[2], goals[2], accelerations[2];
  for (uint8_t i = 0; i < 2; ++i) {
    int32_t cap;
    if (!require(readValue(ids[i], VELOCITY_LIMIT, cap) && cap > 0 && cap <= 1023,
                 "velocity limit")) return;
    profiles[i] = static_cast<int32_t>(floorf(speeds[i] / 1.374f));
    if (profiles[i] < 1 || profiles[i] > cap) {
      Serial.print("REJECTED: motor ID"); Serial.print(ids[i]);
      Serial.print(" speed limit="); Serial.print(cap * 1.374f, 3);
      Serial.println(" deg/s."); return;
    }
  }
  // raw 1 = 214.577 rev/min^2 = 21.4577 deg/s^2.
  // 약 0.3초에 설정 속도에 도달하는 프로파일. 짧은 이동에서는 도달 전에 감속한다.
  for (uint8_t i = 0; i < 2; ++i) {
    const float actual_speed = profiles[i] * 1.374f;
    accelerations[i] = constrain(static_cast<int32_t>(ceilf(actual_speed / (21.4577f * 0.3f))), 1, 32767);
  }
  if (!prepareAxis(0) || !prepareAxis(1)) return;
  for (uint8_t i = 0; i < 2; ++i) {
    const int64_t goal = static_cast<int64_t>(targets[i]) + lroundf(deltas[i] * 4096.0f / 360.0f);
    const int64_t rel = goal - anchors[i];
    if (llabs(rel) > limitTicks(i)) {
      Serial.print("REJECTED: "); Serial.print(i == 0 ? "Y" : "X");
      Serial.print(" result "); Serial.print(rel * 360.0 / 4096.0, 2);
      Serial.print(" deg outside +/-"); Serial.print(angle_limits[i], 0);
      Serial.print("; current "); Serial.print(ticksToDeg(targets[i] - anchors[i]), 2);
      Serial.println(" deg from home.");
      return;
    }
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
  Payload data[2] = {{accelerations[0], profiles[0], goals[0]}, {accelerations[1], profiles[1], goals[1]}};
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
  stage_ms = millis(); sequence = false; dual_move = true; active = true;
  Serial.print("MOVING XY: dX="); Serial.print(values[0], 2);
  Serial.print(" dY="); Serial.print(values[1], 2);
  Serial.print(" -> X="); Serial.print(ticksToDeg(goals[1] - anchors[1]), 2);
  Serial.print(" Y="); Serial.print(ticksToDeg(goals[0] - anchors[0]), 2);
  Serial.print(" deg from home; actual profile X="); Serial.print(dual_speeds[1], 3);
  Serial.print(" Y="); Serial.print(dual_speeds[0], 3); Serial.print(" deg/s; acceleration_raw X=");
  Serial.print(accelerations[1]); Serial.print(" Y="); Serial.println(accelerations[0]);
}

// 메모: 상하 축부터 rel 위치로 이동을 시작한다. 상하 도착 후 좌우 이동은 loop()에서 이어서 실행한다.
void startSequence(const char *name, const int32_t rel[2]) {
  sequence_name = name; sequence_ticks[0] = rel[0]; sequence_ticks[1] = rel[1];
  sequence = moveAxis(0, rel[0], RETURN_SPEED_DEG_S);
}
// 메모: 두 축을 준비하고 홈 위치로 순차 복귀한다.
void startHome() {
  if (!readyForMove()) return;
  if (!prepareAxis(0) || !prepareAxis(1)) return;
  const int32_t home[2] = {0, 0};
  startSequence("HOME", home);
}
// 메모: 시작 시 현재 위치를 저장 위치와 비교한다. 같으면 그대로 두고, 다르면 저장 위치로 순차 이동한다.
void startRestore() {
  if (!saved_valid) { Serial.println("RESTORE: no saved position; waiting for commands."); return; }
  for (uint8_t i = 0; i < 2; ++i) {
    if (!dxl.ping(ids[i])) {
      Serial.print("RESTORE skipped: motor ID"); Serial.print(ids[i]); Serial.println(" no response.");
      return;
    }
  }
  if (!prepareAxis(0) || !prepareAxis(1)) return;
  bool same = true;
  for (uint8_t i = 0; i < 2; ++i) same = same && abs(targets[i] - anchors[i] - saved_ticks[i]) <= RESTORE_TOLERANCE;
  Serial.print(same ? "POSITION OK: " : "POSITION CHANGED: now X=");
  if (!same) {
    Serial.print(ticksToDeg(targets[1] - anchors[1]), 2); Serial.print(" Y=");
    Serial.print(ticksToDeg(targets[0] - anchors[0]), 2); Serial.print(" deg; ");
  }
  Serial.print("saved X="); Serial.print(ticksToDeg(saved_ticks[1]), 2);
  Serial.print(" Y="); Serial.print(ticksToDeg(saved_ticks[0]), 2); Serial.println(" deg from home.");
  if (!same) startSequence("RESTORE", saved_ticks);
}
// 메모: 두 축의 이동을 취소하고 토크를 끈 뒤 watchdog 해제를 요청한다. 무게로 카메라가 내려갈 수 있으므로 먼저 지지해야 한다.
void release() {
  active = false; sequence = false; dual_move = false;
  bool ok = true;
  for (uint8_t i = 0; i < 2; ++i) {
    bool axis_ok = dxl.torqueOff(ids[i]);
    if (axis_ok) axis_ok = dxl.writeControlTableItem(BUS_WATCHDOG, ids[i], 0);
    ok = axis_ok && ok;
    configured[i] = false;
  }
  Serial.println(ok ? "OFF: both axes released. Support camera." : "OFF failed: support camera and check power.");
}
// 별도 진단 스케치 없이 ID와 모터 상태를 확인한다. 설정은 변경하지 않는다.
void scanField(uint8_t id, uint8_t item, const char *name) {
  int32_t value;
  const bool ok = readValue(id, item, value);
  Serial.print(name); Serial.print('=');
  if (ok) Serial.println(value);
  else {
    Serial.print("READ_ERROR lib="); Serial.print(dxl.getLastLibErrCode());
    Serial.print(" status="); Serial.println(dxl.getLastStatusPacketError());
  }
}

void scanMotors() {
  if (active) { Serial.println("BUSY: send x before scan."); return; }
  Serial.println("SCAN_BEGIN: IDs 11 and 12, Protocol 2, 1 Mbps");
  for (uint8_t id = 11; id <= 12; ++id) {
    if (!dxl.ping(id)) {
      Serial.print("NO_RESPONSE id="); Serial.println(id); continue;
    }
    Serial.print("FOUND id="); Serial.print(id);
    Serial.print(" model="); Serial.println(dxl.getModelNumber(id));
    scanField(id, FIRMWARE_VERSION, "firmware");
    scanField(id, PRESENT_POSITION, "position_ticks");
    scanField(id, PRESENT_VELOCITY, "velocity_raw");
    scanField(id, HOMING_OFFSET, "homing_offset");
    scanField(id, TORQUE_ENABLE, "torque_enable");
    scanField(id, HARDWARE_ERROR_STATUS, "hardware_error");
    scanField(id, PRESENT_INPUT_VOLTAGE, "input_voltage_raw_0.1V");
  }
  Serial.println("SCAN_DONE");
}

// 정지 후 현재 위치를 유지하고, 성공하면 그 위치를 저장한다.
void stopAndSave() {
  const bool ok = hold();
  Serial.println("STOP: holding current positions.");
  if (ok && !faulted) savePosition();
}
// Raspberry Pi에서 받은 이동·홈·정지·상태·진단 명령을 처리한다.
void command(char *line) {
  while (isspace(static_cast<unsigned char>(*line))) ++line;
  size_t length = strlen(line);
  while (length && isspace(static_cast<unsigned char>(line[length - 1]))) line[--length] = '\0';
  if (!length) return;
  if (!strcmp(line, "h")) { startHome(); return; }
  if (!strcmp(line, "x")) { stopAndSave(); return; }
  if (!strcmp(line, "off")) { release(); return; }
  if (!strcmp(line, "p")) { printStatus(); return; }
  if (!strcmp(line, "scan")) { scanMotors(); return; }
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
  float common_values[4];
  if (parseCommonMove(line, common_values)) { moveBoth(common_values); return; }
  if (*line == '+' || *line == '-' || *line == '.' || isdigit(static_cast<unsigned char>(*line))) {
    Serial.print("INVALID XY: ["); Serial.print(line); Serial.println("]");
    Serial.print("Input bytes:");
    for (size_t i = 0; i < length; ++i) { Serial.print(' '); Serial.print(static_cast<uint8_t>(line[i]), HEX); }
    Serial.println();
  }
  Serial.println("Use DX_DEG DY_DEG SPEED | h | x | off | p | scan | auto on/off.");
}
// 메모: 보드 시작 시 시리얼·모터 통신을 준비하고 EEPROM의 자동 복귀 설정을 읽는다. auto가 켜져 있으면 홈 복귀를 시작하고, 꺼져 있으면 저장 위치와 비교해 필요하면 복원한다.
void setup() {
  Serial.begin(115200);
  dxl.begin(1000000); dxl.setPortProtocolVersion(2.0);
  BootSetting setting; EEPROM.get(0, setting);
  auto_home = setting.magic == BOOT_MAGIC && setting.enabled == 1 && setting.inverse == 254;
  loadPosition();
  delay(2000);
  float check[4];
  if (!parseCommonMove("10 10 10", check) || check[0] != 10 || check[1] != 10 ||
      check[2] != 10 || check[3] != 10) {
    faulted = true; reason = "XY parser self-check";
    Serial.println("FAULT: XY parser self-check failed."); return;
  }
  Serial.print(CONTROL_BUILD_ID);
  Serial.println(": numeric input parser OK; DX_DEG DY_DEG SPEED.");
  Serial.println("OPENCR_READY: moves relative to current position; home range pan +/-180, tilt +/-135.");
  if (auto_home) startHome(); else startRestore();
}
// 메모: 시리얼 입력과 즉시 정지 x를 처리하고 100ms마다 오류·위치·속도를 확인한다. 도착하면 다음 홈 축을 실행하거나 토크를 유지하며 이동을 완료한다.
void loop() {
  static char line[80]; static uint8_t used = 0; static bool overflow = false;
  for (uint8_t n = 0; n < 32 && Serial.available(); ++n) {
    char c = Serial.read();
    if (c == 'x') { stopAndSave(); used = 0; overflow = false; continue; }
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
      savePosition();
    }
    return;
  }
  const uint32_t timeout_ms = 15000 + static_cast<uint32_t>(
      ceilf(1000 * 2 * angle_limits[stage] / (floorf(stage_speed / 1.374f) * 1.374f)));
  if (!require(millis() - stage_ms < timeout_ms, "movement timeout")) return;
  int32_t p, v;
  if (!require(readValue(ids[stage], PRESENT_POSITION, p) && readValue(ids[stage], PRESENT_VELOCITY, v), "feedback")) return;
  settled = (llabs(static_cast<int64_t>(p) - targets[stage]) <= 8 && abs(v) <= 1) ? settled + 1 : 0;
  if (settled < 5) return;
  Serial.print("REACHED "); Serial.println(stage == 0 ? "tilt" : "pan");
  active = false;
  if (sequence && stage == 0) { if (!moveAxis(1, sequence_ticks[1], stage_speed)) sequence = false; }
  else {
    sequence = false;
    Serial.print("DONE "); Serial.print(sequence_name); Serial.println(": holding position; no automatic torque off.");
    savePosition();
  }
}
