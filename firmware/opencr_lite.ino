// OpenCR / XM430-W350: pan ID11, tilt ID12.
// USB at 115200: v PAN TILT (rad/s), p (read), x (stop).
// Reply: P1 SAMPLE PAN_TICKS TILT_TICKS FAULT\n.
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t AXES = 2;
constexpr uint32_t CYCLE_US = 10000;  // 100 Hz target
constexpr uint32_t COMMAND_TIMEOUT_MS = 200;
constexpr float SPEED_UNIT = 1.374f * PI / 180.0f;
constexpr float MAX_RAD_S = 1.0f;
constexpr int32_t MAX_POSITION = 1048575;
constexpr int32_t TRAVEL_TOLERANCE = 8;

namespace Reg {
constexpr uint16_t DRIVE = 10, MODE = 11, HOME_OFFSET = 20;
constexpr uint16_t SPEED_LIMIT = 44, TORQUE = 64, ERROR = 70;
constexpr uint16_t WATCHDOG = 98, ACCELERATION = 108;
constexpr uint16_t PROFILE = 112, GOAL = 116, POSITION = 132;
}

struct Axis {
  uint8_t id;
  int32_t home, travel;
  int32_t position, origin, speed_limit, applied;
  int32_t profile, goal;
  bool ready;
};

Axis axes[AXES] = {{11, 0, 2048}, {12, 2048, 1536}};
int32_t requested[AXES] = {};
bool faulted = false, command_active = false;
uint32_t command_ms = 0, cycle_us = 0, sample = 0;

Dynamixel2Arduino motors(Serial3, 84);
ParamForSyncReadInst_t read_packet;
ParamForSyncWriteInst_t write_packet;
RecvInfoFromStatusInst_t replies;

// Motor I/O. Every response must be complete and error-free.
bool responseOK() {
  return motors.getLastLibErrCode() == DXL_LIB_OK &&
    motors.getLastStatusPacketError() == 0;
}

bool readMotor(uint8_t i, uint16_t addr, void *data, uint16_t size) {
  return motors.read(axes[i].id, addr, size,
    (uint8_t *)data, size, 2) == size && responseOK();
}

bool writeMotor(uint8_t i, uint16_t addr, int32_t value, uint16_t size) {
  return motors.write(axes[i].id, addr,
    (uint8_t *)&value, size, 2) && responseOK();
}

bool configure(uint8_t i, uint16_t addr, int32_t value, uint16_t size) {
  int32_t actual = 0;
  if (!readMotor(i, addr, &actual, size)) return false;
  if (actual == value) return true;
  return writeMotor(i, addr, value, size) &&
    readMotor(i, addr, &actual, size) && actual == value;
}

bool readBoth(uint16_t addr, uint16_t size) {
  read_packet.addr = addr;
  read_packet.length = size;
  if (!motors.syncRead(read_packet, replies, 2) ||
      replies.id_count != AXES) return false;
  for (uint8_t i = 0; i < AXES; ++i) {
    const auto &reply = replies.xel[i];
    if (reply.id != axes[i].id || reply.error || reply.length != size)
      return false;
  }
  return true;
}

// Read measured position and check the motor failsafes each cycle.
bool readPosition() {
  if (!readBoth(Reg::POSITION, 4)) return false;
  for (uint8_t i = 0; i < AXES; ++i) {
    memcpy(&axes[i].position, replies.xel[i].data, 4);
    int64_t relative = (int64_t)axes[i].position - axes[i].origin;
    if (llabs(relative) > axes[i].travel + TRAVEL_TOLERANCE)
      return false;
  }
  if (!readBoth(Reg::TORQUE, 7)) return false;
  for (uint8_t i = 0; i < AXES; ++i) {
    const uint8_t *data = replies.xel[i].data;
    if (data[0] != 1 || data[Reg::ERROR - Reg::TORQUE]) return false;
  }
  if (!readBoth(Reg::WATCHDOG, 1)) return false;
  for (uint8_t i = 0; i < AXES; ++i)
    if (replies.xel[i].data[0] == 255) return false;
  return true;
}

void stopMotion() {
  command_active = false;
  for (uint8_t i = 0; i < AXES; ++i) {
    Axis &axis = axes[i];
    int32_t position;
    requested[i] = axis.applied = 0;
    if (axis.ready && readMotor(i, Reg::POSITION, &position, 4) &&
        writeMotor(i, Reg::GOAL, position, 4)) {
      axis.goal = position;
    } else {
      faulted = true;
      writeMotor(i, Reg::TORQUE, 0, 1);
    }
  }
}

void latchFault() {
  faulted = true;  // restart the board to clear a fault
  stopMotion();
}

// Speed magnitude sets the profile; direction selects the travel boundary.
// Extended Position mode decelerates and holds at that bounded target.
bool writeSpeed() {
  bool changed = false;
  for (uint8_t i = 0; i < AXES; ++i)
    changed |= requested[i] != axes[i].applied;
  if (!changed) return true;
  for (uint8_t i = 0; i < AXES; ++i) {
    Axis &axis = axes[i];
    int32_t speed = requested[i];
    if (speed != axis.applied) {
      axis.profile = speed == 0 ? 1 : abs(speed);
      axis.goal = speed == 0 ? axis.position :
        axis.origin + (speed > 0 ? axis.travel : -axis.travel);
    }
    // Profile zero means unlimited speed, so zero commands hold position.
    int32_t data[] = {axis.profile, axis.goal};
    memcpy(write_packet.xel[i].data, data, sizeof(data));
  }
  if (!motors.syncWrite(write_packet) || !readBoth(Reg::PROFILE, 8))
    return false;
  for (uint8_t i = 0; i < AXES; ++i) {
    if (memcmp(replies.xel[i].data, write_packet.xel[i].data, 8))
      return false;
    axes[i].applied = requested[i];
  }
  return true;
}

bool prepareMotor(uint8_t i) {
  Axis &axis = axes[i];
  int32_t offset = 0, error = 0, drive = 0;
  if (!motors.ping(axis.id) || motors.getModelNumber(axis.id) != XM430_W350)
    return false;
  if (!configure(i, Reg::TORQUE, 0, 1) ||
      !configure(i, Reg::WATCHDOG, 0, 1) ||
      !readMotor(i, Reg::HOME_OFFSET, &offset, 4) || offset != 0 ||
      !readMotor(i, Reg::ERROR, &error, 1) || error != 0 ||
      !readMotor(i, Reg::DRIVE, &drive, 1)) return false;
  // Preserve direction; clear time-profile and torque-on-by-goal bits.
  if (!configure(i, Reg::DRIVE, drive & ~12, 1) ||
      !configure(i, Reg::MODE, 4, 1) ||
      !readMotor(i, Reg::SPEED_LIMIT, &axis.speed_limit, 4) ||
      axis.speed_limit < 1 || axis.speed_limit > 1023 ||
      !readMotor(i, Reg::POSITION, &axis.position, 4)) return false;
  int32_t relative = ((int64_t)axis.position - axis.home) % 4096;
  if (relative > 2048) relative -= 4096;
  if (relative < -2048) relative += 4096;
  if (abs(relative) > axis.travel) return false;
  int64_t origin = (int64_t)axis.position - relative;
  if (llabs(origin) + axis.travel > MAX_POSITION) return false;
  axis.origin = origin;
  axis.profile = 1;
  axis.goal = axis.position;
  axis.ready = configure(i, Reg::ACCELERATION, 8, 4) &&
    configure(i, Reg::PROFILE, axis.profile, 4) &&
    configure(i, Reg::GOAL, axis.goal, 4) &&
    configure(i, Reg::TORQUE, 1, 1) &&
    configure(i, Reg::WATCHDOG, COMMAND_TIMEOUT_MS / 20, 1);
  return axis.ready;
}

// Serial interface: p reads position; v writes angular speed; x stops.
void sendPosition() {
  char line[80];
  int length = snprintf(line, sizeof(line), "P1 %lu %ld %ld %u\n",
    (unsigned long)sample,
    (long)((int64_t)axes[0].position - axes[0].origin),
    (long)((int64_t)axes[1].position - axes[1].origin),
    (unsigned)faulted);
  if (Serial && length > 0 && length < (int)sizeof(line))
    Serial.write((uint8_t *)line, length);
}

void command(char *line) {
  if (!strcmp(line, "p")) { sendPosition(); return; }
  if (!strcmp(line, "x")) { stopMotion(); return; }
  if (faulted) return;
  if (line[0] != 'v' || line[1] != ' ') { stopMotion(); return; }
  char *cursor = line + 2, *end;
  int32_t speeds[AXES];
  for (uint8_t i = 0; i < AXES; ++i) {
    float speed = strtof(cursor, &end);
    if (cursor == end || !isfinite(speed) || fabsf(speed) > MAX_RAD_S ||
        fabsf(speed) > axes[i].speed_limit * SPEED_UNIT) {
      stopMotion();
      return;
    }
    speeds[i] = speed / SPEED_UNIT;
    cursor = end;
    if (i == 0 && *cursor != ' ') { stopMotion(); return; }
  }
  while (*cursor == ' ' || *cursor == '\t') ++cursor;
  if (*cursor) { stopMotion(); return; }
  memcpy(requested, speeds, sizeof(speeds));
  command_ms = millis();
  command_active = true;
}

void readCommands() {
  static char line[64];
  static uint8_t used = 0;
  static bool discard = false;
  for (uint8_t n = 0; n < 64 && Serial.available(); ++n) {
    char c = Serial.read();
    if (c == '\n') {
      line[used] = 0;
      if (!discard) command(line);
      used = 0;
      discard = false;
    } else if (c != '\r' && !discard) {
      if (c < 32 || c > 126 || used == sizeof(line) - 1) {
        discard = true;
        stopMotion();
      } else {
        line[used++] = c;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  motors.begin(1000000);
  motors.setPortProtocolVersion(2.0);
  read_packet.id_count = write_packet.id_count = AXES;
  write_packet.addr = Reg::PROFILE;
  write_packet.length = 8;
  for (uint8_t i = 0; i < AXES; ++i)
    read_packet.xel[i].id = write_packet.xel[i].id = axes[i].id;
  delay(1000);
  if (!prepareMotor(0) || !prepareMotor(1)) latchFault();
}

void loop() {
  if (command_active &&
      (!Serial || millis() - command_ms >= COMMAND_TIMEOUT_MS))
    stopMotion();
  readCommands();
  if (faulted || (uint32_t)(micros() - cycle_us) < CYCLE_US) return;
  cycle_us = micros();
  if (!readPosition() || !writeSpeed()) { latchFault(); return; }
  ++sample;
}
