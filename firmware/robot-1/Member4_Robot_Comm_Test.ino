/*
  =============================================================================
   BATTLEBOT - ROBOT-SIDE COMMUNICATIONS MODULE (Member 4)
  =============================================================================
   Platform : ESP32 (Arduino framework)
   Role     : Wi-Fi access point + UDP link between the handheld controller
             and the robot.
 
   Data flow:
       Controller --[ControlPacket, UDP :4210]--> Robot  (joystick + buttons)
       Robot      --[TelemetryPacket, UDP :4211]--> Controller (status data)
 
   Design notes:
    - The robot hosts its own Wi-Fi network (no external router needed), so the link works anywhere, including competition arenas.
    - UDP is used instead of TCP: it is connectionless and has no retransmission delay. For real-time control, a fresh packet is always
      more useful than a reliably-delivered old one.
    - A failsafe timeout detects a dead link so the robot can stop safely.
 
   Status: TEST HARNESS. Health, battery, sensors and motor control are
   stubbed out and will be merged in from the other team members' modules.
 =============================================================================
 */


#include <WiFi.h> // ESP32 Wi-Fi stack (access point / station modes)
#include <WiFiUdp.h>  // UDP socket wrapper
// -----------------------------------------------------------------------------
//  NETWORK CONFIGURATION
// -----------------------------------------------------------------------------

const char* AP_SSID     = "BATTLEBOT_R1";
const char* AP_PASSWORD = "battlebit1";
IPAddress apIP(192, 168, 4, 1);  
IPAddress apGateway(192, 168, 4, 1); // Robot is its own gateway
IPAddress apSubnet(255, 255, 255, 0); // /24 network (254 usable addresses)

// Separate ports per direction keep the two data streams cleanly separated.
const uint16_t CONTROL_PORT   = 4210;
const uint16_t TELEMETRY_PORT = 4211;

// -----------------------------------------------------------------------------
//  TIMING CONFIGURATION
// -----------------------------------------------------------------------------
const unsigned long LINK_TIMEOUT_MS = 400;
const unsigned long TELEMETRY_INTERVAL_MS = 100;

// -----------------------------------------------------------------------------
//  RUNTIME STATE
// -----------------------------------------------------------------------------

WiFiUDP udp; // Single UDP socket used for RX and TX
IPAddress remoteIP; // Controller's address
bool remoteIPKnown = false; // False until a controller has contacted
unsigned long lastPacketTime = 0; // millis() timestamp of last valid RX
unsigned long lastTelemetryTime = 0; // millis() timestamp of last TX

// Controller -> Robot: operator input.
struct ControlPacket {
  uint32_t seq;
  int16_t  joyX;
  int16_t  joyY;
  uint8_t  pb1;
  uint8_t  pb2;
  uint8_t  pb3;
  uint8_t  pb4;
};

// Robot -> Controller: status report shown on the operator's display.
struct TelemetryPacket {
  uint32_t seq;
  int16_t  health;
  float    batteryVoltage;
  float    angleX;
  float    angleY;
  uint8_t  magnetState;
  uint8_t  servoState;
  uint8_t  robotState;
  uint8_t  linkOK;
};

int16_t fakeHealth = 100; // stand-in until Member 3's code is merged in

// =============================================================================
//  SETUP: runs once at boot
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP started: "); Serial.println(AP_SSID);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  udp.begin(CONTROL_PORT);
  Serial.println("Member 4 (Robot side) comms test ready.");
}

void loop() {
  handleIncoming();
  checkFailsafe();
  sendTelemetryIfDue();
}

void handleIncoming() {
  int packetSize = udp.parsePacket(); // Returns 0 if nothing has arrived
  if (packetSize == sizeof(ControlPacket)) {
    ControlPacket pkt;
    udp.read((uint8_t*)&pkt, sizeof(pkt));
    remoteIP = udp.remoteIP();
    remoteIPKnown = true;
    lastPacketTime = millis();

    Serial.print("RX seq="); Serial.print(pkt.seq);
    Serial.print(" joyX="); Serial.print(pkt.joyX);
    Serial.print(" joyY="); Serial.print(pkt.joyY);
    Serial.print(" pb1-4="); Serial.print(pkt.pb1);
    Serial.print(pkt.pb2); Serial.print(pkt.pb3); Serial.println(pkt.pb4);
  } else if (packetSize > 0) {
    udp.flush();
  }
}

void checkFailsafe() {
  if (remoteIPKnown && (millis() - lastPacketTime > LINK_TIMEOUT_MS)) {
    Serial.println("LINK LOST - (would stop motors here)");
  }
}

void sendTelemetryIfDue() {
  unsigned long now = millis();
  if (!remoteIPKnown) return; // No one to send to yet
  if (now - lastTelemetryTime < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryTime = now;

  fakeHealth = max(0, fakeHealth - 0); // placeholder, real health comes from Member 3

  TelemetryPacket t;
  t.seq = 0;
  t.health = fakeHealth;
  t.batteryVoltage = 11.1; // fake value for testing
  t.angleX = 0;
  t.angleY = 0;
  t.magnetState = 0;
  t.servoState = 0;
  t.robotState = 0;
  t.linkOK = (now - lastPacketTime <= LINK_TIMEOUT_MS) ? 1 : 0;
  
// Transmit the struct as raw bytes to the controller's telemetry port.
  udp.beginPacket(remoteIP, TELEMETRY_PORT);
  udp.write((uint8_t*)&t, sizeof(t));
  udp.endPacket();
}
