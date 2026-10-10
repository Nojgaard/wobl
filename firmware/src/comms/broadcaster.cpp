#include "comms/broadcaster.hpp"
#include "common/event_timer.hpp"
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP _udp;
static EventTimer timeout;
static bool establishedConnection = false;

static constexpr const char *NVS_NAMESPACE = "wobl_wifi";
static constexpr const char *NVS_KEY_SSID = "ssid";
static constexpr const char *NVS_KEY_PASSWORD = "pwd";

// Fixed-size buffers: an SSID is <=32 bytes, a WPA2 passphrase <=63.
static char ssid[33] = {0};
static char password[64] = {0};

static constexpr uint16_t UDP_PORT = 8888;
static IPAddress broadcastIP = IPAddress(192, 168, 4, 255);
static unsigned long lastSendTime = 0;

struct BroadcastTelemetry {
  uint32_t timestampMs;

  // Input Commands
  bool cmdEnable;
  float cmdRoll;
  float cmdHeight;
  float cmdFwdVel;
  float cmdTurnVel;

  // Observed State
  float obsPitch;
  float obsPitchRate;
  float obsRoll;
  float obsRollRate;
  float obsFwdVel;
  float obsTurnVel;
  float obsLeftLegHeight;
  float obsRightLegHeight;

  // Control Output
  float ctrlWheelLeft;
  float ctrlWheelRight;
  float ctrlServoLeft;
  float ctrlServoRight;
} __attribute__((packed));

bool loadCredentials() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, true))
    return false;

  if (!prefs.isKey(NVS_KEY_SSID) || !prefs.isKey(NVS_KEY_PASSWORD))
    return false;

  prefs.getString(NVS_KEY_SSID, ssid, sizeof(ssid));
  prefs.getString(NVS_KEY_PASSWORD, password, sizeof(password));
  return true;
}

bool Broadcaster::saveSsid(const char *newSsid) {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false))
    return false;

  return prefs.putString(NVS_KEY_SSID, newSsid) > 0;
}

bool Broadcaster::savePassword(const char *newPassword) {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false))
    return false;

  return prefs.putString(NVS_KEY_PASSWORD, newPassword) > 0;
}

void Broadcaster::init() { WiFi.mode(WIFI_OFF); }

void Broadcaster::enable(bool on) {
  if (on == _enabled)
    return;

  if (on) {
    if (!loadCredentials()) {
      Serial.printf("[broadcaster] failed to load wifi credentials\n");
      return;
    }

    Serial.printf("[broadcaster] connecting to '%s'...\n", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(ssid, password);

    timeout.init();
    establishedConnection = false;
  } else {
    _udp.stop();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("[broadcaster] disabled");
  }
  _enabled = on;
}

void Broadcaster::update() {
  if (!_enabled)
    return;

  if (WiFi.status() != WL_CONNECTED) {
    if (!timeout.poll(EventTimer::Duration::fromMs(5000)).blank()) {
      Serial.printf("[broadcaster] Connection Failed. Status=%i\n",
                    WiFi.status());
      enable(false);
    }
    return;
  }

  if (!establishedConnection) {
    Serial.printf("[broadcaster] Connected to '%s'  IP: %s\n", ssid,
                  WiFi.localIP().toString().c_str());
    broadcastIP = WiFi.broadcastIP();
    establishedConnection = true;
  }

  auto tel = _robot.controller.telemetry();
  if (tel.timestampMs == lastSendTime)
    return;

  lastSendTime = tel.timestampMs;
  BroadcastTelemetry bt;
  bt.timestampMs = tel.timestampMs;

  // Input Commands
  bt.cmdEnable = tel.command.enable;
  bt.cmdRoll = tel.command.roll;
  bt.cmdHeight = tel.command.height;
  bt.cmdFwdVel = tel.command.forwardVelocity;
  bt.cmdTurnVel = tel.command.turnVelocity;

  // Observed State
  bt.obsPitch = tel.state.pitch;
  bt.obsPitchRate = tel.state.pitchRate;
  bt.obsRoll = tel.state.roll;
  bt.obsRollRate = tel.state.rollRate;
  bt.obsFwdVel = tel.state.forwardVelocity;
  bt.obsTurnVel = tel.state.turnVelocity;
  bt.obsLeftLegHeight = tel.state.leftLegHeight;
  bt.obsRightLegHeight = tel.state.rightLegHeight;

  // Control Output
  bt.ctrlWheelLeft = tel.output.wheels.left.velocity;
  bt.ctrlWheelRight = tel.output.wheels.right.velocity;
  bt.ctrlServoLeft = tel.output.servos.left.positionRad;
  bt.ctrlServoRight = tel.output.servos.right.positionRad;

  _udp.beginPacket(broadcastIP, UDP_PORT);
  _udp.write((const uint8_t *)&bt, sizeof(bt));
  _udp.endPacket();
}