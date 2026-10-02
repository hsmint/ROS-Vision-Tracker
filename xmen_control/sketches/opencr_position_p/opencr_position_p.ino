// Lv2 3강 — P 제어: 휠 모드 + 엔코더 피드백으로 위치 제어.
// 수강생이 튜닝하는 Kp는 OpenCR의 위치 제어 게인이다.
// 목표각도 - 엔코더 각도 -> P 계산 -> Goal Velocity -> 모터 -> 엔코더.
// 다이나믹셀은 내부 속도 PI만 사용한다. 내부 위치 PID는 사용하지 않는다.
// XM430-W350, Protocol 2.0, ID 12, 1 Mbps, firmware >= 38, 모터 1개.
// 실행: s <Kp> <speed_deg_s|max> <angle_deg>; x 정지. 시작 위치가 0도, 2초 후 입력 목표로 이동.
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

using namespace ControlTableItem;
Dynamixel2Arduino dxl(Serial3, 84);  // OpenCR DXL 포트 / 방향 제어 핀

const uint8_t DXL_ID = 12;          // 사용자 모터 검색 결과
const uint32_t DXL_BAUD = 1000000;
constexpr float PI_F = 3.14159265358979323846f;
constexpr float RAD_PER_DEG = PI_F / 180.0f;
constexpr float RAD_PER_TICK = 2.0f * PI_F / 4096.0f;
constexpr float RAD_S_PER_VELOCITY_RAW = 0.229f * 2.0f * PI_F / 60.0f;
constexpr float DEFAULT_SPEED_RAD_S = 20.0f * RAD_PER_DEG;
constexpr float MAX_ANGLE_DEG = 90.0f;
constexpr float DEADBAND_RAD = 0.2f * RAD_PER_DEG;
const uint32_t PERIOD_US = 10000;    // 100 Hz 요청 주기; 실제 dt는 로그로 확인
uint32_t run_ms = 60000;  // 시작할 때 입력 각도·속도로 결정한다. 최소 60초.

bool running = false;
bool faulted = false;
const char *fault_reason = "unknown";
float kp = 0.5f;                    // [1/s], 모터 내부 Position P Gain과 다름
float ki = 0.0f, kd = 0.0f;
double p_term = 0, i_term = 0, d_term = 0;
float speed_limit_rad_s = DEFAULT_SPEED_RAD_S;
float goal_rad = 90.0f * RAD_PER_DEG;
int32_t origin_ticks = 0;
int32_t velocity_limit_raw = 0;  // 모터의 EEPROM 한계를 읽어 사용한다.
uint32_t started_ms = 0, last_us = 0, last_log_ms = 0;

// 메모: 부호 있는 속도를 -limit부터 +limit 사이로 제한한다.
constexpr float limitSpeed(float speed, float limit) {
  return speed > limit ? limit : speed < -limit ? -limit : speed;
}

// 메모: rad/s 속도를 모터의 Goal Velocity raw 단위로 바꾼다. 사용자 속도와 모터 한계를 모두 적용하며 반올림 후에도 상한을 넘지 않게 한다.
int32_t velocityToRaw(float speed, float limit, int32_t motor_limit_raw) {
  // 큰 입력도 정수로 바꾸기 전에 모터가 허용하는 범위로 제한한다.
  const int32_t raw_limit = static_cast<int32_t>(fminf(limit / RAD_S_PER_VELOCITY_RAW, motor_limit_raw));
  const float raw_speed = limitSpeed(speed / RAD_S_PER_VELOCITY_RAW, raw_limit);
  return constrain(static_cast<int32_t>(lroundf(raw_speed)),
                   -raw_limit, raw_limit);  // 반올림 후에도 입력한 명령 상한을 넘지 않는다.
}

// PID/입력 검사는 ../check_feedback.py 참조.

// P/I는 위치 오차, D는 측정 속도를 사용해 목표 변경 순간의 미분 급증을 피한다.
// 메모: 로그용 P·I·D 항을 0으로 초기화한다. 현재 시험에서는 I·D를 사용하지 않는다.
void resetPid() { p_term = i_term = d_term = 0; }
// P 제어: 현재 위치 오차에 비례하는 속도를 명령한다. I와 D는 항상 0.
// 메모: 위치 오차에 OpenCR의 Kp를 곱해 목표 각속도를 계산한다. 작은 오차는 deadband로 제거하고 출력 속도를 제한한다. 모터 내부 위치 PID와 다른 외부 P 제어다.
double positionControl(float error_rad, float /*measured_speed_rad_s*/, float /*dt_s*/,
                   double output_min, double output_max) {
  const float error = fabsf(error_rad) < DEADBAND_RAD ? 0.0f : error_rad;
  p_term = static_cast<double>(kp) * error;
  i_term = d_term = 0;
  return fmax(output_min, fmin(output_max, p_term));
}

// 메모: 각도와 속도로 시험 제한 시간을 계산한다. 최소 60초이며, 아주 느린 입력은 정수 범위를 넘지 않도록 최대 시간을 제한한다.
uint32_t trialDurationMs(float angle_rad, float speed_rad_s) {
  const double estimate = 12000.0 + ceil(3000.0 * fabs(angle_rad) / speed_rad_s);
  // 아주 작은 속도에서도 타이머 정수 변환이 넘치지 않게 한다(최대 약 24.9일).
  return static_cast<uint32_t>(fmin(2147483647.0, fmax(60000.0, estimate)));
}
// 메모: 설정 종류에 맞게 값을 검사한다. k는 유한성 검사 후 허용하고, v는 양수, a는 -90~90도만 허용한다.
constexpr bool validSetting(char key, float value) {
  return key == 'k' ? true :
         key == 'v' ? value > 0 && value * RAD_PER_DEG > 0 :
         key == 'a' ? value >= -MAX_ANGLE_DEG && value <= MAX_ANGLE_DEG : false;
}
// 메모: 입력의 max 토큰을 찾고 토큰 뒤 위치를 end에 전달한다. 다른 단어에 포함된 max는 허용하지 않는다.
bool parseMaxSpeed(const char *text, const char *&end) {
  while (isspace(static_cast<unsigned char>(*text))) ++text;
  if (strncmp(text, "max", 3) != 0 ||
      (text[3] != '\0' && !isspace(static_cast<unsigned char>(text[3])))) return false;
  end = text + 3;
  return true;
}
// 메모: k/v/a 개별 설정 명령을 읽고 값과 후행 문자열을 검사한다. v max를 허용하며 이 P 시험에서는 i/d 설정을 거부한다.
bool parseSettingCommand(const char *line, float &value) {
  if (line[0] != 'k' && line[0] != 'i' && line[0] != 'd' &&
      line[0] != 'v' && line[0] != 'a') return false;
  const char *max_end;
  if (line[0] == 'v' && parseMaxSpeed(line + 1, max_end)) {
    while (isspace(static_cast<unsigned char>(*max_end))) ++max_end;
    if (*max_end != '\0') return false;
    value = INFINITY; return true;
  }
  char *end;
  value = strtof(line + 1, &end);
  if (end == line + 1) return false;
  while (isspace(static_cast<unsigned char>(*end))) ++end;
  return *end == '\0' && isfinite(value) && validSetting(line[0], value);
}
// 메모: s Kp 속도 각도 한 줄을 검사한다. 모든 값이 유효할 때만 출력 인자를 갱신하며 Ki·Kd는 항상 0으로 설정한다.
bool parseRunCommand(const char *line, float &pg, float &ig, float &dg,
                     float &speed_deg_s, float &angle_deg) {
  if (line[0] != 's' || !isspace(static_cast<unsigned char>(line[1]))) return false;
  const char *cursor = line + 1;
  const char keys[] = "kva";
  float values[3];
  for (uint8_t i = 0; i < 3; ++i) {
    const char *max_end;
    if (i == 1 && parseMaxSpeed(cursor, max_end)) {
      values[i] = INFINITY; cursor = max_end; continue;
    }
    char *end;
    values[i] = strtof(cursor, &end);
    if (end == cursor || !isfinite(values[i]) || !validSetting(keys[i], values[i])) return false;
    if (*end != '\0' && !isspace(static_cast<unsigned char>(*end))) return false;
    cursor = end;
  }
  while (isspace(static_cast<unsigned char>(*cursor))) ++cursor;
  if (*cursor != '\0') return false;
  pg = values[0]; ig = 0; dg = 0;
  speed_deg_s = values[1]; angle_deg = values[2];
  return true;  // 모든 입력이 유효할 때에만 적용한다.
}
// 메모: 정상·오류 입력 예제를 검사해 파서가 예상대로 동작하는지 확인한다. 모터를 움직이는 시험이 아니다.
bool settingsSelfCheck() {
  float value, pg = 9, ig = 9, dg = 9, speed = 20, angle = 90;
  return parseSettingCommand("k -100", value) && value == -100
      && parseSettingCommand("v max", value) && isinf(value)
      && !parseSettingCommand("k nan", value)
      && !parseSettingCommand("v inf", value)
      && !parseSettingCommand("a 91", value)
      && !parseSettingCommand("i 1", value)
      && !parseSettingCommand("d 1", value)
      && !parseRunCommand("s 1 20 91", pg, ig, dg, speed, angle)
      && pg == 9 && ig == 9 && dg == 9 && speed == 20 && angle == 90
      && !parseRunCommand("s 1 20 90 extra", pg, ig, dg, speed, angle)
      && parseRunCommand("s 1 max -90", pg, ig, dg, speed, angle)
      && pg == 1 && ig == 0 && dg == 0
      && isinf(speed) && angle == -90;
}

// 메모: 현재 Kp·Ki·Kd, 최대 속도와 목표 상대각도를 시리얼에 출력한다.
void printSettings() {
  Serial.print("P ");
  Serial.print("SET Kp="); Serial.print(kp, 4);
  Serial.print(", Ki="); Serial.print(ki, 4);
  Serial.print(", Kd="); Serial.print(kd, 4);
  Serial.print(", speed_limit_deg_s=");
  if (isinf(speed_limit_rad_s)) Serial.print("max");
  else Serial.print(speed_limit_rad_s / RAD_PER_DEG, 3);
  Serial.print(", angle_deg="); Serial.println(goal_rad / RAD_PER_DEG, 3);
}

// 메모: 시험을 중단하고 속도 0·토크 OFF를 요청한다. 오류가 있으면 모터 버스 전원도 차단하고 RESET 전까지 재시작을 막는다. 카메라가 처질 수 있다.
void stopRun(const char *reason, bool fault) {
  running = false;
  resetPid();
  // 읽기 실패를 위치 0으로 해석하지 않는다. 정지 패킷은 각각 한 번 보낸다.
  const bool zero_ok = dxl.setGoalVelocity(DXL_ID, 0, UNIT_RPM);
  const bool off_ok = dxl.torqueOff(DXL_ID);
  faulted = fault || !zero_ok || !off_ok;
  if (faulted) fault_reason = reason;
  if (faulted) {
    // OpenCR에서 전원을 공급받는 DXL 포트를 차단한다. RESET 전까지 재시작 금지.
    digitalWrite(BDPIN_DXL_PWR_EN, LOW);
  }
  Serial.print(faulted ? "FAULT (RESET required): " : "STOP: ");
  Serial.println(reason);
}

// 메모: 실패한 모터 작업의 오류 코드를 보존하고 stopRun()을 호출한 뒤 오류를 출력한다.
bool requireOk(bool ok, const char *reason) {
  if (!ok) {
    const auto lib_error = dxl.getLastLibErrCode();
    const auto status_error = dxl.getLastStatusPacketError();
    stopRun(reason, true);  // 정지 통신이 원래 오류를 덮어쓰기 전에 위에서 보존한다.
    Serial.print("DXL lib_error="); Serial.print(lib_error);
    Serial.print(", status_error="); Serial.println(status_error);
  }
  return ok;
}

// 메모: 현재 DXL_ID의 제어 테이블 값을 읽고 통신·모터 상태 오류를 검사한다. 읽기 실패는 정상 위치 0으로 취급하지 않는다.
bool readItem(uint8_t item, int32_t &value) {
  value = dxl.readControlTableItem(item, DXL_ID, 10);  // timeout [ms]
  return requireOk(dxl.getLastLibErrCode() == DXL_LIB_OK &&
                   dxl.getLastStatusPacketError() == 0, "DXL read failed");
}

// 메모: 속도 프로파일과 watchdog을 준비하고 토크를 켠 뒤 현재 엔코더를 새 0도로 저장한다. 시험 시간과 제어 주기를 초기화해 실행을 시작한다.
void startRun() {
  if (running || faulted) return;
  // max에서는 예제의 완만한 가속 프로파일을 해제한다. 숫자 속도는 기존 raw 5.
  if (!requireOk(dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID,
                                         isinf(speed_limit_rad_s) ? 0 : 5), "acceleration")) return;
  // watchdog 오류를 지우고, 남아 있는 속도 명령을 0으로 만든 뒤 토크를 켠다.
  if (!requireOk(dxl.writeControlTableItem(BUS_WATCHDOG, DXL_ID, 0), "watchdog clear") ||
      !requireOk(dxl.setGoalVelocity(DXL_ID, 0, UNIT_RPM), "zero velocity") ||
      !requireOk(dxl.torqueOn(DXL_ID), "torque on") ||
      !requireOk(dxl.writeControlTableItem(BUS_WATCHDOG, DXL_ID, 5), "watchdog set")) return;
  if (!readItem(PRESENT_POSITION, origin_ticks)) return;
  resetPid();
  run_ms = trialDurationMs(goal_rad, speed_limit_rad_s);
  started_ms = millis();
  last_log_ms = started_ms;
  last_us = micros();
  running = true;
  Serial.println("START: current position = 0 deg.");
  printSettings();
  Serial.print("Run timeout [s]: "); Serial.println(run_ms / 1000.0f, 1);
}

// 메모: 시리얼 한 줄 입력으로 설정·시험 시작을 처리한다. x는 줄바꿈 없이 즉시 정지하고, max 토큰 안의 x는 정지로 해석하지 않는다.
void readCommands() {
  static char line[96];
  static uint8_t used = 0;
  static bool overflow = false;
  for (uint8_t n = 0; n < 32 && Serial.available(); ++n) {
    const char c = Serial.read();
    // max 토큰의 마지막 x와 즉시 정지 명령 x를 구분한다.
    const bool max_token_end = !overflow && used >= 2 && line[used - 2] == 'm' &&
        line[used - 1] == 'a' && (used == 2 || isspace(static_cast<unsigned char>(line[used - 3])));
    if (c == 'x' && !max_token_end) {  // 정지는 줄바꿈을 기다리지 않는다.
      used = 0; overflow = false;
      if (running) stopRun("user", false);
      continue;
    }
    if (c == '\r' || c == '\n') {
      line[used] = '\0';
      float value, pg, ig, dg, speed, angle;
      if (overflow) Serial.println("Command too long; discarded.");
      else if (used && faulted) {
        Serial.print("FAULT: RESET required. Reason: ");
        Serial.println(fault_reason);
      }
      else if (used && running) Serial.println("Running: send x before changing settings.");
      else if (used && strcmp(line, "s") == 0) startRun();
      else if (used && parseRunCommand(line, pg, ig, dg, speed, angle)) {
        kp = pg; ki = ig; kd = dg;
        speed_limit_rad_s = speed * RAD_PER_DEG; goal_rad = angle * RAD_PER_DEG;
        startRun();
      } else if (used && parseSettingCommand(line, value)) {
        if (line[0] == 'k') kp = value;
        if (line[0] == 'i') ki = value;
        if (line[0] == 'd') kd = value;
        if (line[0] == 'v') speed_limit_rad_s = value * RAD_PER_DEG;
        if (line[0] == 'a') goal_rad = value * RAD_PER_DEG;
        printSettings();
      } else if (used) {
        Serial.println("Invalid setting. Finite Kp, positive speed or max, angle -90..90. Use k/v/a or s <Kp> <speed_deg_s|max> <angle_deg>.");
      }
      used = 0; overflow = false;
    } else if (!overflow) {
      if (used < sizeof(line) - 1) line[used++] = c;
      else overflow = true;
    }
  }
}

// 메모: 모터 ID·모델·펌웨어·Drive Mode를 확인하고 속도 모드 및 속도 한계를 준비한다. 모든 검사가 통과하면 READY를 출력한다.
void setup() {
  Serial.begin(115200);
  dxl.begin(DXL_BAUD);  // 라이브러리가 OpenCR의 DXL 전원도 켠다.
  dxl.setPortProtocolVersion(2.0);
  if (!requireOk(dxl.ping(DXL_ID), "ping: check ID/baud/power") ||
      !requireOk(dxl.getModelNumber(DXL_ID) == XM430_W350, "requires XM430-W350") ||
      !requireOk(dxl.torqueOff(DXL_ID), "torque off")) return;
  if (!requireOk(settingsSelfCheck(), "settings parser self-check")) return;
  int32_t value;
  if (!readItem(FIRMWARE_VERSION, value) ||
      !requireOk(value >= 38, "requires firmware >= 38")) return;
  if (!readItem(DRIVE_MODE, value) ||
      !requireOk((value & 4) == 0, "use velocity-based profile: Drive Mode bit 2 = 0")) return;
  if (!readItem(OPERATING_MODE, value)) return;
  if (value != OP_VELOCITY &&
      !requireOk(dxl.setOperatingMode(DXL_ID, OP_VELOCITY), "velocity mode")) return;
  // EEPROM 속도 한계는 읽기만 한다. 가속 프로파일은 시작할 때 max 여부로 정한다.
  if (!readItem(VELOCITY_LIMIT, velocity_limit_raw) ||
      !requireOk(velocity_limit_raw >= 0 && velocity_limit_raw <= 1023, "Velocity Limit")) return;
  Serial.println("READY: s <Kp> <speed_deg_s|max> <angle_deg>; k/v/a set; s start; x stop. Newline.");
  Serial.println("Kp: any finite value; speed: positive or max (motor limits).");
  Serial.print("Motor velocity raw cap: "); Serial.println(velocity_limit_raw);
  printSettings();
}

// 메모: 약 100Hz로 엔코더를 읽어 상대 위치 오차를 계산하고 P 제어 속도를 쓴다. 10Hz 로그를 출력하며 지연·오류·시간 제한에서는 시험을 멈춘다.
void loop() {
  readCommands();
  if (!running) return;
  const uint32_t now_us = micros();
  const uint32_t dt_us = now_us - last_us;  // unsigned 차분: micros() wrap 대응
  if (dt_us < PERIOD_US) return;
  if (dt_us > 5 * PERIOD_US) { stopRun("control loop late", true); return; }
  last_us = now_us;
  const uint32_t elapsed_ms = millis() - started_ms;
  if (elapsed_ms >= run_ms) { stopRun("time limit reached; torque off", false); return; }

  // 1. 엔코더 위치와 속도를 읽는다. 속도는 로그에 사용한다.
  int32_t ticks, measured_velocity_raw;
  if (!readItem(PRESENT_POSITION, ticks) || !readItem(PRESENT_VELOCITY, measured_velocity_raw)) return;
  const float measured_speed_rad_s = measured_velocity_raw * RAD_S_PER_VELOCITY_RAW;
  // 32비트 signed 연속 위치를 차분한다. 0/360도 경계에서 각도를 접지 않는다.
  const float position_rad = static_cast<float>(static_cast<int64_t>(ticks) - origin_ticks)
                             * RAD_PER_TICK;

  // 2. 시작 위치를 0도로 본 상대각과 목표각도의 차이를 구한다.
  const float target_rad = elapsed_ms < 2000 ? 0.0f : goal_rad;
  const float error_rad = target_rad - position_rad;
  const int32_t max_velocity_raw = velocityToRaw(speed_limit_rad_s, speed_limit_rad_s, velocity_limit_raw);
  const float output_limit = max_velocity_raw * RAD_S_PER_VELOCITY_RAW;
  // 3. 수강생이 입력한 Kp로 위치 P를 계산한다. 결과는 목표 각속도다.
  const float speed_rad_s = positionControl(error_rad, measured_speed_rad_s, dt_us * 1e-6f,
                                        -output_limit, output_limit);
  // 4. 휠 모드에 부호 있는 속도를 전달한다. 목표를 지나치면 반대 방향 보정도 가능하다.
  const int32_t velocity_raw = velocityToRaw(speed_rad_s, speed_limit_rad_s, velocity_limit_raw);
  if (!requireOk(dxl.setGoalVelocity(DXL_ID, velocity_raw, UNIT_RAW),
                 "velocity write failed")) return;

  // ponytail: 동기식 단일 모터 폴링. 다축/더 빠른 주기가 필요하면 Sync Read/Write로 변경.
  if (millis() - last_log_ms >= 100 && Serial) {  // 10 Hz 로그, Arduino Serial Plotter
    last_log_ms = millis();
    Serial.print("target_deg:"); Serial.print(target_rad / RAD_PER_DEG, 3);
    Serial.print("\tposition_deg:"); Serial.print(position_rad / RAD_PER_DEG, 3);
    Serial.print("\terror_deg:"); Serial.print(error_rad / RAD_PER_DEG, 3);
    Serial.print("\tp_deg_s:"); Serial.print(p_term / RAD_PER_DEG, 3);
    Serial.print("\ti_deg_s:"); Serial.print(i_term / RAD_PER_DEG, 3);
    Serial.print("\td_deg_s:"); Serial.print(d_term / RAD_PER_DEG, 3);
    Serial.print("\tpid_deg_s:"); Serial.print((p_term + i_term + d_term) / RAD_PER_DEG, 3);
    Serial.print("\tspeed_deg_s:"); Serial.print(measured_speed_rad_s / RAD_PER_DEG, 3);
    Serial.print("\tu_deg_s:"); Serial.print(velocity_raw * RAD_S_PER_VELOCITY_RAW / RAD_PER_DEG, 3);
    Serial.print("\tv_limit_deg_s:"); Serial.print(isinf(speed_limit_rad_s) ? -1.0f : speed_limit_rad_s / RAD_PER_DEG, 3);
    Serial.print("\tdt_ms:"); Serial.print(dt_us / 1000.0f, 3);
    Serial.print("\tkp:"); Serial.print(kp, 4);
    Serial.print("\tki:"); Serial.print(ki, 4);
    Serial.print("\tkd:"); Serial.print(kd, 4);
    Serial.print("\tt_s:"); Serial.println(elapsed_ms / 1000.0f, 3);
  }
}
