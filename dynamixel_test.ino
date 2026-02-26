#include <Dynamixel2Arduino.h>

#if defined(ARDUINO_OpenRB)
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;
#else
#error "Select OpenRB-150 board in Tools > Board"
#endif

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

const float PROTOCOL_VER = 1.0;
const uint32_t DXL_BAUD = 1000000;

const uint8_t IDS[] = { 1, 2, 3 };
const uint8_t N = sizeof(IDS) / sizeof(IDS[0]);

// limits for motion
const uint16_t MIN_POS = 250;
const uint16_t MAX_POS = 750;

// Speed setup
const uint16_t BASE_SPEED = 100;
const uint16_t MIN_SPEED = 10;
const uint16_t MAX_SPEED = 120;

void centerAllMotors(uint16_t speed = MAX_SPEED) {
  DEBUG_SERIAL.println("Centering");

  // Set a gentle speed for all motors
  for (uint8_t i = 0; i < N; i++) {
    dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, IDS[i], speed);
  }

  // Send center position
  for (uint8_t i = 0; i < N; i++) {
    dxl.setGoalPosition(IDS[i], 512);
  }
}

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) {}

  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(PROTOCOL_VER);

  // Ping + torque on all motors
  for (uint8_t i = 0; i < N; i++) {
    uint8_t id = IDS[i];
    DEBUG_SERIAL.print("Pinging ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print("... ");
    if (!dxl.ping(id)) {
      DEBUG_SERIAL.println("FAIL");
      while (1) {}
    }
    DEBUG_SERIAL.println("OK");

    dxl.torqueOff(id);
    dxl.torqueOn(id);
  }

  randomSeed(analogRead(A0));  

  DEBUG_SERIAL.println("All motors ready.");

  centerAllMotors();

  delay(500);  
}

bool anyMoving() {
  for (uint8_t i = 0; i < N; i++) {
    uint8_t id = IDS[i];
    uint8_t moving = dxl.readControlTableItem(ControlTableItem::MOVING, id);
    if (moving != 0) return true;
  }
  return false;
}

uint16_t clampSpeed(uint16_t s) {
  if (s < MIN_SPEED) return MIN_SPEED;
  if (s > MAX_SPEED) return MAX_SPEED;
  return s;
}

void loop() {
  uint16_t present[N];
  uint16_t goal[N];
  uint16_t dist[N];
  if (anyMoving()) return;

  for (uint8_t i = 0; i < N; i++) {
    uint8_t id = IDS[i];
    present[i] = (uint16_t)dxl.getPresentPosition(id);  
    goal[i] = random(MIN_POS, MAX_POS + 1);
    dist[i] = (present[i] > goal[i]) ? (present[i] - goal[i]) : (goal[i] - present[i]);
  }

  // 2) Find the maximum distance
  uint16_t maxDist = 0;
  for (uint8_t i = 0; i < N; i++) {
    if (dist[i] > maxDist) maxDist = dist[i];
  }
  if (maxDist == 0) maxDist = 1;  // avoid divide by zero if all already at goal

  // 3) Set per-motor speed so they finish together
  for (uint8_t i = 0; i < N; i++) {
    uint8_t id = IDS[i];

    // proportional speed (farthest motor uses BASE_SPEED)
    uint16_t spd = (uint32_t)BASE_SPEED * dist[i] / maxDist;
    spd = clampSpeed(spd);

    dxl.writeControlTableItem(ControlTableItem::MOVING_SPEED, id, spd);

    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" present=");
    DEBUG_SERIAL.print(present[i]);
    DEBUG_SERIAL.print(" goal=");
    DEBUG_SERIAL.print(goal[i]);
    DEBUG_SERIAL.print(" dist=");
    DEBUG_SERIAL.print(dist[i]);
    DEBUG_SERIAL.print(" speed=");
    DEBUG_SERIAL.println(spd);
  }

  // 4) Send all goal positions
  for (uint8_t i = 0; i < N; i++) {
    dxl.setGoalPosition(IDS[i], goal[i]);
  }

}