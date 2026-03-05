#include <Dynamixel2Arduino.h>

#if !defined(ARDUINO_OpenRB)
  #error "Select OpenRB-150 board in Tools > Board"
#endif

#define DXL_SERIAL Serial1
#define USB_SERIAL Serial
static constexpr int DXL_DIR_PIN = -1;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ---- DYNAMIXEL ----
static constexpr float    PROTOCOL_VER = 1.0;
static constexpr uint32_t DXL_BAUD     = 1000000;

static constexpr uint8_t  IDS[] = {1, 2, 3}; // num of motors
static constexpr uint8_t  N     = sizeof(IDS) / sizeof(IDS[0]); // total num of motors

static constexpr uint16_t POS_MAX = 1023;
static constexpr uint16_t SPD_MIN = 1;
static constexpr uint16_t SPD_MAX = 1023;

// ---- Protocol ----
// [AA][55][LEN][CMD][payload...][CRC_L][CRC_H]
static constexpr uint8_t STX1 = 0xAA;
static constexpr uint8_t STX2 = 0x55;

enum Command : uint8_t {
  CMD_SET_GOAL   = 0x01,  // [id][pos_lo][pos_hi][spd_lo][spd_hi]
  CMD_SET_MULTI  = 0x02,  // [count][(id,pos,spd)*count]
  CMD_STOP_ALL   = 0x03,  // no payload
  CMD_CENTER_ALL = 0x04,  // no payload
};

// ---- Timing ----
static constexpr uint16_t APPLY_HZ        = 100;  
static constexpr uint32_t APPLY_PERIOD_MS = 1000 / APPLY_HZ;
static constexpr uint32_t WATCHDOG_MS     = 200; // Stop applying old command after this delay without msg
static constexpr bool     TORQUE_OFF_ON_TIMEOUT = false;

// ---- State ----
bool     pending[N] = {false};
uint16_t goalPos[N] = {512, 512, 512};
uint16_t goalSpd[N] = {80,  80,  80};
uint32_t lastCmdMs  = 0;

// ---- Helpers ----
static uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static bool findIdIndex(uint8_t id, uint8_t &idxOut) {
  for (uint8_t i = 0; i < N; i++) {
    if (IDS[i] == id) { idxOut = i; return true; }
  }
  return false;
}

static uint16_t crc16_modbus(const uint8_t* buf, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
  }
  return crc;
}

static void centerAll(uint16_t speed = 80) {
  speed = clampU16(speed, SPD_MIN, SPD_MAX);
  for (uint8_t i = 0; i < N; i++) {
    dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, IDS[i], speed);
    dxl.setGoalPosition(IDS[i], 512);
    pending[i] = false;
  }
  lastCmdMs = millis();
}

static void stopAllHold() {
  for (uint8_t i = 0; i < N; i++) {
    const uint8_t id = IDS[i];
    goalPos[i] = clampU16((uint16_t)dxl.getPresentPosition(id), 0, POS_MAX);
    goalSpd[i] = SPD_MIN;
    pending[i] = true;
  }
  lastCmdMs = millis();
}

// ---- RX parser ----
static constexpr uint8_t RX_BUF_SZ = 64;
uint8_t rx[RX_BUF_SZ];

enum class RxState : uint8_t { WaitSTX1, WaitSTX2, WaitLEN, ReadBody };
RxState rxState = RxState::WaitSTX1;
uint8_t rxPos = 0;
uint8_t expectedTotal = 0;
uint8_t lastPacketTotal = 0;

static bool parseOnePacket() {
  while (USB_SERIAL.available() > 0) {
    const uint8_t b = (uint8_t)USB_SERIAL.read();

    switch (rxState) {
      case RxState::WaitSTX1:
        if (b == STX1) { rx[0] = b; rxPos = 1; rxState = RxState::WaitSTX2; }
        break;

      case RxState::WaitSTX2:
        if (b == STX2) { rx[1] = b; rxPos = 2; rxState = RxState::WaitLEN; }
        else { rxState = RxState::WaitSTX1; rxPos = 0; }
        break;

      case RxState::WaitLEN: {
        rx[2] = b;
        rxPos = 3;
        const uint8_t len = b;
        expectedTotal = 2 + 1 + len + 2;
        if (expectedTotal < 6 || expectedTotal > RX_BUF_SZ) {
          rxState = RxState::WaitSTX1; rxPos = 0; expectedTotal = 0;
        } else {
          rxState = RxState::ReadBody;
        }
      } break;

      case RxState::ReadBody:
        rx[rxPos++] = b;
        if (expectedTotal && rxPos >= expectedTotal) {
          const uint16_t got  = (uint16_t)rx[expectedTotal - 2] | ((uint16_t)rx[expectedTotal - 1] << 8);
          const uint16_t calc = crc16_modbus(&rx[2], (uint16_t)(1 + rx[2]));

          rxState = RxState::WaitSTX1;
          rxPos = 0;

          if (got == calc) {
            lastPacketTotal = expectedTotal;
            expectedTotal = 0;
            return true;
          }
          expectedTotal = 0; // drop bad CRC
        }
        break;
    }
  }
  return false;
}

// ---- Command handling ----
static void handleSetGoal() {
  if (rx[2] != (1 + 5)) return;

  const uint8_t id = rx[4];
  uint16_t pos = (uint16_t)rx[5] | ((uint16_t)rx[6] << 8);
  uint16_t spd = (uint16_t)rx[7] | ((uint16_t)rx[8] << 8);

  uint8_t idx;
  if (!findIdIndex(id, idx)) return;

  goalPos[idx] = clampU16(pos, 0, POS_MAX);
  goalSpd[idx] = clampU16((spd == 0) ? 1 : spd, SPD_MIN, SPD_MAX);
  pending[idx] = true;
  lastCmdMs = millis();
}

static void handleSetMulti() {
  const uint8_t len = rx[2];
  if (len < 2) return;

  const uint8_t count = rx[4];
  const uint8_t expectedLen = (uint8_t)(1 + 1 + count * 5); // cmd + count + records
  if (len != expectedLen) return;

  const uint8_t expectedTotalBytes = (uint8_t)(2 + 1 + len + 2);
  if (expectedTotalBytes != lastPacketTotal) return;

  uint8_t p = 5;
  for (uint8_t k = 0; k < count; k++) {
    const uint8_t id = rx[p++];
    uint16_t pos = (uint16_t)rx[p++] | ((uint16_t)rx[p++] << 8);
    uint16_t spd = (uint16_t)rx[p++] | ((uint16_t)rx[p++] << 8);

    uint8_t idx;
    if (!findIdIndex(id, idx)) continue;

    goalPos[idx] = clampU16(pos, 0, POS_MAX);
    goalSpd[idx] = clampU16((spd == 0) ? 1 : spd, SPD_MIN, SPD_MAX);
    pending[idx] = true;
  }
  lastCmdMs = millis();
}

static void dispatchPacket() {
  switch (rx[3]) {
    case CMD_SET_GOAL:   handleSetGoal();  break;
    case CMD_SET_MULTI:  handleSetMulti(); break;
    case CMD_STOP_ALL:   stopAllHold();    break;
    case CMD_CENTER_ALL: centerAll(80);    break;
    default: break;
  }
}

static void drainSerialKeepLatest() {
  while (parseOnePacket()) dispatchPacket();
}

// ---- Apply (2-pass to reduce stagger) ----
static void applyLatestIfDue() {
  static uint32_t lastApplyMs = 0;
  const uint32_t now = millis();
  if (now - lastApplyMs < APPLY_PERIOD_MS) return;
  lastApplyMs = now;

  if (now - lastCmdMs > WATCHDOG_MS) {
    if (TORQUE_OFF_ON_TIMEOUT) {
      for (uint8_t i = 0; i < N; i++) dxl.torqueOff(IDS[i]);
    }
    for (uint8_t i = 0; i < N; i++) pending[i] = false;
    return;
  }

  // Snapshot which motors need updating
  bool doAny = false;
  bool todo[N];
  for (uint8_t i = 0; i < N; i++) {
    todo[i] = pending[i];
    doAny |= todo[i];
  }
  if (!doAny) return;

  // Pass 1: write all speeds
  for (uint8_t i = 0; i < N; i++) {
    if (!todo[i]) continue;
    dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, IDS[i], goalSpd[i]);
  }

  // Pass 2: write all goal positions (this is the “start” trigger)
  for (uint8_t i = 0; i < N; i++) {
    if (!todo[i]) continue;
    dxl.setGoalPosition(IDS[i], goalPos[i]);
    pending[i] = false;
  }
}

// ---- Arduino ----
void setup() {
  USB_SERIAL.begin(115200);
  while (!USB_SERIAL) {}

  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(PROTOCOL_VER);

  for (uint8_t i = 0; i < N; i++) {
    if (!dxl.ping(IDS[i])) while (1) {}
    dxl.torqueOff(IDS[i]);
    dxl.torqueOn(IDS[i]);
  }

  lastCmdMs = millis();
  centerAll(80);
}

void loop() {
  drainSerialKeepLatest();
  applyLatestIfDue();
}