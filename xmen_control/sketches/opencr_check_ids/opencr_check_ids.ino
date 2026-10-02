#include <Dynamixel2Arduino.h>
#include <string.h>
using namespace ControlTableItem;

// Only PING and READ packets. No motor configuration or motion commands.
Dynamixel2Arduino dxl(Serial3, 84);
const uint32_t bauds[] = {57600, 1000000, 115200, 9600,
                          2000000, 3000000, 4000000, 4500000};

// 메모: 진단 펌웨어 배너와 사용 안내를 출력한다. 아래 안내 문자열은 원본의 단일 모터 문구지만 실제 scan()은 ID11과 ID12를 각각 확인한다.
void help() {
  Serial.println("OPENCR_DIAGNOSTIC_V1: PING/READ only");
  Serial.println("Connect ONE motor. Type scan then Enter (up to 60 s).");
  Serial.println("Type help to show this banner. No motion/settings writes.");
}

// 메모: 모터의 항목 하나를 읽어 이름=값 형식으로 출력한다. 읽기 오류를 정상 측정값 0과 구분해 오류 코드도 출력한다.
void field(uint8_t id, uint8_t item, const char *name) {
  const int32_t value = dxl.readControlTableItem(item, id, 30);
  const auto error = dxl.getLastLibErrCode();
  const auto status = dxl.getLastStatusPacketError();
  Serial.print(name); Serial.print("=");
  if (error != DXL_LIB_OK || status != 0) {
    Serial.print("READ_ERROR lib="); Serial.print(error);
    Serial.print(" status="); Serial.println(status);
  } else {
    Serial.println(value);
  }
}

// 메모: 1Mbps·Protocol 2.0에서 ID11과 ID12에 PING을 보내고 모델·위치·속도·토크·전압·오류를 읽는다. 모터 설정 변경이나 구동 명령은 보내지 않는다.
void scan() {
  Serial.println("SCAN_BEGIN: IDs 11 and 12, Protocol 2, 1 Mbps");
  dxl.setPortProtocolVersion(2.0);
  dxl.begin(1000000);
  delay(100);
  for (uint8_t id = 11; id <= 12; ++id) {
    if (!dxl.ping(id)) {
      Serial.print("NO_RESPONSE id="); Serial.println(id);
      continue;
    }
    Serial.print("FOUND id="); Serial.print(id);
    Serial.print(" baud=1000000 protocol=2 model=");
    Serial.println(dxl.getModelNumber(id));
    field(id, FIRMWARE_VERSION, "firmware");
    field(id, PRESENT_POSITION, "home_position_ticks");
    field(id, PRESENT_VELOCITY, "velocity_raw");
    field(id, HOMING_OFFSET, "homing_offset");
    field(id, TORQUE_ENABLE, "torque_enable");
    field(id, HARDWARE_ERROR_STATUS, "hardware_error");
    field(id, PRESENT_INPUT_VOLTAGE, "input_voltage_raw_0.1V");
  }
  Serial.println("SCAN_DONE: both IDs checked; settings unchanged.");
}

// 메모: USB 시리얼과 모터 버스를 초기화하고 안내를 출력한다. 이때 57600으로 시작하지만 scan()에서 버스 속도를 1Mbps로 바꾼다.
void setup() {
  Serial.begin(115200);
  dxl.begin(57600);
  help();
}

// 메모: 줄바꿈까지 입력을 모아 scan 명령이면 조회를 수행한다. 나머지 명령은 안내를 출력하며, 너무 긴 입력은 버린다.
void loop() {
  static char line[24];
  static uint8_t used = 0;
  static bool overflow = false;
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\r' || c == '\n') {
      line[used] = '\0';
      if (overflow) Serial.println("Command too long; use help or scan.");
      else if (used && strcmp(line, "scan") == 0) scan();
      else if (used) help();
      used = 0; overflow = false;
    } else if (!overflow) {
      if (used < sizeof(line) - 1) line[used++] = c;
      else overflow = true;
    }
  }
}
