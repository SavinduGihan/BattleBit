/**
* =====================================================================
* MEMBER 3 - SENSORS & HEALTH SYSTEM (standalone test sketch)
* Robot 1 - MPU6050 body-angle + Magnetic Reed Switch + Health/Dead logic
* =====================================================================

 * ============================================================================
 *  Project   : Combat Robot - Robot 1
 *  Module    : Sensors & Health System (Standalone Test Sketch)
 *  Component : MPU6050 (tilt sensing) + Magnetic Reed Switch (hit detection)
 * ============================================================================
 *
 *  OVERVIEW
 *  --------
 *  This sketch validates the MPU6050 inertial sensor and the reed switch
 *  independently, and verifies the health/dead-state logic over the Serial
 *  Monitor before integration into the main Robot1.ino firmware.
 *
 *  SCOPE
 *  -----
 *  Wi-Fi, motor control, and actuator code are intentionally excluded so
 *  that sensor and health behavior can be tested in isolation.
 *
 *  HEALTH RULES
 *  ------------
 *  - Reed switch hit : Deducts DAMAGE_PER_HIT. A cooldown (REED_COOLDOWN_MS)
 *                      prevents multiple deductions from a single trigger.
 *  - Excessive tilt  : If the body angle exceeds TILT_ANGLE_LIMIT_DEG for
 *                      longer than TILT_PENALTY_HOLD_MS, TILT_DAMAGE is
 *                      applied once per tilt event.
 *  - Death           : When health reaches 0, the robot enters the DEAD
 *                      state and the red indicator LED turns on.
 *
 *  TEST PROCEDURE
 *  --------------
 *  1. Upload the sketch and open the Serial Monitor at 115200 baud.
 *  2. Tilt the robot beyond TILT_ANGLE_LIMIT_DEG and hold it. After
 *     TILT_PENALTY_HOLD_MS, a single damage event should occur and health
 *     should decrease by TILT_DAMAGE.
 *  3. Trigger the reed switch. Health should decrease by DAMAGE_PER_HIT
 *     once per trigger (cooldown prevents repeated hits).
 *  4. Continue until health reaches 0 and verify the DEAD state is reported
 *     and the dead-state LED activates.
 *  5. Send 'r' via the Serial Monitor at any time to reset health.
 *
 *  DEPENDENCIES
 *  ------------
 *  - Adafruit MPU6050
 *  - Adafruit Unified Sensor
 *
 *  OPEN ITEMS (Concept Document, Section 17)
 *  -----------------------------------------
 *  - Confirm starting health value and damage per reed-switch hit.
 *  - Confirm MPU6050 tilt threshold, hold time, and tilt damage.
 *  - Confirm reed switch active level and pull resistor configuration.
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------------------------------------------------------------------------
// Pin Definitions
// ---------------------------------------------------------------------------
#define REED_SWITCH_PIN 34   // Reed switch input (TODO: confirm active level / pull resistor)
#define LED_DEAD_PIN    2    // Red LED indicating dead state

// ---------------------------------------------------------------------------
// Sensor Objects & Status
// ---------------------------------------------------------------------------
Adafruit_MPU6050 mpu;
bool mpuOK = false;          // True if MPU6050 initialized successfully

// ---------------------------------------------------------------------------
// Health & Reed Switch Configuration
// ---------------------------------------------------------------------------
const int16_t HEALTH_MAX = 100;              // TODO: confirm starting health
const int16_t DAMAGE_PER_HIT = 15;           // TODO: confirm damage per hit
const unsigned long REED_COOLDOWN_MS = 400;  // Debounce / cooldown between hits
unsigned long lastReedTriggerTime = 0;       // Timestamp of last registered hit

// ---------------------------------------------------------------------------
// Tilt Penalty Configuration
// ---------------------------------------------------------------------------
const float TILT_ANGLE_LIMIT_DEG = 45.0;         // TODO: confirm tilt threshold
const unsigned long TILT_PENALTY_HOLD_MS = 1500; // TODO: confirm hold time
const int16_t TILT_DAMAGE = 5;                   // TODO: confirm tilt damage
unsigned long tiltStartTime = 0;                 // When the current tilt began
bool tiltPenaltyApplied = false;                 // Ensures one penalty per tilt event

// ---------------------------------------------------------------------------
// Runtime State
// ---------------------------------------------------------------------------
int16_t health = HEALTH_MAX;
bool robotDead = false;

unsigned long lastPrint = 0;  // Timestamp of last Serial status print

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(REED_SWITCH_PIN, INPUT);
  pinMode(LED_DEAD_PIN, OUTPUT);
  digitalWrite(LED_DEAD_PIN, LOW);

  // Initialize MPU6050 and configure measurement ranges / filtering
  Wire.begin();
  mpuOK = mpu.begin();
  if (mpuOK) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 OK");
  } else {
    Serial.println("MPU6050 NOT FOUND - check wiring/address");
  }

  Serial.println("Member 3 - Sensors & Health test ready. Type 'r' to reset health.");
}

void loop() {
  // Serial command: 'r' resets health and clears the dead state
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      health = HEALTH_MAX;
      robotDead = false;
      Serial.println(">>> HEALTH RESET <<<");
    }
  }

  updateHealthSystem();
  updateDeadIndicator();

  // Periodic status report (every 500 ms)
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    float ax, ay;
    float tilt = readTiltAngleDeg(ax, ay);
    Serial.print("HP: "); Serial.print(health);
    Serial.print("  Tilt: "); Serial.print(tilt, 1);
    Serial.print(" deg  State: "); Serial.println(robotDead ? "DEAD" : "ALIVE");
  }
}

// Computes body tilt from accelerometer data.
// Returns the larger of the X/Y axis angles (in degrees); individual
// axis angles are returned via the reference parameters.
float readTiltAngleDeg(float &angleX, float &angleY) {
  if (!mpuOK) { angleX = 0; angleY = 0; return 0; }
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  angleX = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  angleY = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / PI;
  return max(fabs(angleX), fabs(angleY));
}

// Subtracts health and triggers the dead state when health reaches zero.
// Ignored if the robot is already dead.
void applyDamage(int16_t amount) {
  if (robotDead) return;
  health -= amount;
  Serial.print(">>> DAMAGE EVENT: -"); Serial.println(amount);
  if (health <= 0) {
    health = 0;
    robotDead = true;
    Serial.println(">>> ROBOT DEAD <<<");
  }
}

// Evaluates damage sources each cycle: reed switch hits and sustained tilt.
void updateHealthSystem() {
  if (robotDead) return;

  // Reed switch hit detection with cooldown
  bool reedActive = (digitalRead(REED_SWITCH_PIN) == HIGH); // TODO: confirm active level
  unsigned long now = millis();
  if (reedActive && (now - lastReedTriggerTime > REED_COOLDOWN_MS)) {
    lastReedTriggerTime = now;
    applyDamage(DAMAGE_PER_HIT);
  }

  // Tilt penalty: applied once if the limit is exceeded for the hold time
  float ax, ay;
  float tilt = readTiltAngleDeg(ax, ay);
  if (tilt > TILT_ANGLE_LIMIT_DEG) {
    if (tiltStartTime == 0) tiltStartTime = now;
    if (!tiltPenaltyApplied && (now - tiltStartTime > TILT_PENALTY_HOLD_MS)) {
      applyDamage(TILT_DAMAGE);
      tiltPenaltyApplied = true;
    }
  } else {
    tiltStartTime = 0;
    tiltPenaltyApplied = false;
  }
}

// Drives the dead-state LED to match the robot's current state.
void updateDeadIndicator() {
  digitalWrite(LED_DEAD_PIN, robotDead ? HIGH : LOW);
}
