#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_OpenRB)
  #define DXL_SERIAL   Serial1
  #define USB_SERIAL   Serial      // PC <-> OpenRB over USB
  const int DXL_DIR_PIN = -1;
#else
  #error "Select OpenRB-150 board in Tools > Board"
#endif

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

const float    PROTOCOL_VER = 1.0;
const uint32_t DXL_BAUD = 1000000;

const uint8_t IDS[] = {1, 2, 3};
const uint8_t N = sizeof(IDS) / sizeof(IDS[0]);

// ---- Tuning knobs ----
const uint16_t APPLY_HZ = 50;               // Dynamixel write rate
const uint32_t APPLY_PERIOD_MS = 1000 / APPLY_HZ;
const uint32_t WATCHDOG_MS = 200;           // if no command this long -> stop applying
const bool     TORQUE_OFF_ON_TIMEOUT = false; // if true, torque off when timed out

// Cached "latest" commands
volatile bool     pending[N] = {false, false, false};
volatile uint16_t goalPos[N] = {512, 512, 512};
volatile uint16_t goalSpd[N] = {80,  80,  80};
volatile uint32_t lastCmdMs  = 0;

bool isValidId(uint8_t id, uint8_t &idxOut) {
  for (uint8_t i = 0; i < N; i++) {
    if (IDS[i] == id) { idxOut = i; return true; }
  }
  return false;
}

// ---- CRC16 (Modbus polynomial 0xA001) ----
uint16_t crc16_modbus(const uint8_t* buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else { crc >>= 1; }
    }
  }
  return crc;
}

// ---- Packet parser (non-blocking) ----
static const uint8_t STX1 = 0xAA;
static const uint8_t STX2 = 0x55;

uint8_t rxBuf[64];
uint8_t rxPos = 0;
uint8_t expectedTotal = 0;

// Returns true if a complete valid packet was parsed into rxBuf
bool tryParseOnePacket() {
  while (USB_SERIAL.available() > 0) {
    uint8_t b = (uint8_t)USB_SERIAL.read();

    if (rxPos == 0) {                 // look for STX1
      if (b == STX1) rxBuf[rxPos++] = b;
      continue;
    }
    if (rxPos == 1) {                 // look for STX2
      if (b == STX2) rxBuf[rxPos++] = b;
      else rxPos = 0;
      continue;
    }

    if (rxPos == 2) {                 // LEN
      rxBuf[rxPos++] = b;
      uint8_t len = b;
      expectedTotal = 2 + 1 + len + 2; // stx(2) + len(1) + (cmd+payload=len) + crc(2)
      if (expectedTotal > sizeof(rxBuf) || expectedTotal < 6) { // minimum sanity
        rxPos = 0; expectedTotal = 0;
      }
      continue;
    }

    // rest of packet
    rxBuf[rxPos++] = b;

    if (expectedTotal && rxPos >= expectedTotal) {
      // Validate CRC
      uint16_t got  = (uint16_t)rxBuf[expectedTotal - 2] | ((uint16_t)rxBuf[expectedTotal - 1] << 8);
      uint16_t calc = crc16_modbus(&rxBuf[2], 1 + rxBuf[2]); // LEN + (CMD+payload)

      // Reset state for next packet before returning
      rxPos = 0;
      expectedTotal = 0;

      if (got == calc) return true;
      // CRC bad -> ignore, continue parsing remaining bytes
    }
  }
  return false;
}

void cachePacketAsLatest() {
  uint8_t len = rxBuf[2];
  uint8_t cmd = rxBuf[3];

  if (cmd != 0x01) return;           // only one command for now
  if (len != (1 + 5)) return;        // cmd + payload size

  uint8_t id = rxBuf[4];
  uint16_t pos = (uint16_t)rxBuf[5] | ((uint16_t)rxBuf[6] << 8);
  uint16_t spd = (uint16_t)rxBuf[7] | ((uint16_t)rxBuf[8] << 8);

  uint8_t idx;
  if (!isValidId(id, idx)) return;

  if (pos > 1023) pos = 1023;
  if (spd == 0) spd = 1;
  if (spd > 1023) spd = 1023;

  goalPos[idx] = pos;
  goalSpd[idx] = spd;
  pending[idx] = true;
  lastCmdMs = millis();
}

void drainSerialKeepLatest() {
  bool gotAny = false;
  // Keep parsing packets until no more complete packets are available.
  while (tryParseOnePacket()) {
    cachePacketAsLatest();
    gotAny = true;
  }
  (void)gotAny;
}

void applyLatestIfDue() {
  static uint32_t lastApplyMs = 0;
  uint32_t now = millis();

  if (now - lastApplyMs < APPLY_PERIOD_MS) return;
  lastApplyMs = now;

  // Watchdog: if the PC stopped sending, stop applying updates
  if (now - lastCmdMs > WATCHDOG_MS) {
    if (TORQUE_OFF_ON_TIMEOUT) {
      for (uint8_t i = 0; i < N; i++) dxl.torqueOff(IDS[i]);
    }
    // Clear pending so old cached values don't apply later
    for (uint8_t i = 0; i < N; i++) pending[i] = false;
    return;
  }

  // Apply any pending latest goals
  for (uint8_t i = 0; i < N; i++) {
    if (!pending[i]) continue;
    pending[i] = false;

    uint8_t id = IDS[i];
    dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, id, goalSpd[i]);
    dxl.setGoalPosition(id, goalPos[i]);
  }
}

void centerAllMotors(uint16_t speed = 80) {
  for (uint8_t i = 0; i < N; i++) {
    dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, IDS[i], speed);
    dxl.setGoalPosition(IDS[i], 512);
  }
}

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
  centerAllMotors();
}

void loop() {
  // 1) Drain serial fast, keeping only latest commands
  drainSerialKeepLatest();

  // 2) Apply latest cached commands at fixed rate + watchdog
  applyLatestIfDue();
}