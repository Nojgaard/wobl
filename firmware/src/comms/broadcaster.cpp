#include "comms/broadcaster.hpp"
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP _udp;

static constexpr const char *kApSsid = "wobl";
static constexpr const char *kApPassword = "wobl1234";
static constexpr uint16_t kUdpPort = 8888;
static IPAddress kUdpAddress = IPAddress(192, 168, 4, 255);
static unsigned long lastSendTime = 0;

struct BroadcastTelemetry {
    uint32_t timestamp_ms;       
    float    pitch;             
    float    pitchRate;          
    float    roll;              
    float    rollRate;           
    float    wheelVelLeft;       
    float    wheelVelRight;      
    float    bodyFwdVelocity;    
    float    bodyYawRate;        
    float    targetFwdVelocity;  
    float    targetYawRate;      
    float    wheelCmdLeft;   
    float    wheelCmdRight;
} __attribute__((packed));

void Broadcaster::init() {}

void Broadcaster::enable(bool on) {
  if (on && !_enabled) {
    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(kApSsid, kApPassword);
    _udp.begin(kUdpPort);
    Serial.printf("[broadcaster] AP '%s' on port %u  IP: %s  (softAP=%s)\n",
                  kApSsid, kUdpPort, WiFi.softAPIP().toString().c_str(),
                  ok ? "OK" : "FAILED");
  } else if (!on && _enabled) {
    _udp.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("[broadcaster] disabled");
  }
  _enabled = on;
}

void Broadcaster::update() {
  if (!_enabled)
    return;

  auto tel = _robot.controller.telemetry();
  if (tel.timestampMs == lastSendTime)
    return;

  auto wtel = _robot.wheels.telemetry();
  lastSendTime = tel.timestampMs;
  BroadcastTelemetry bt;
  bt.timestamp_ms = tel.timestampMs;
  bt.pitch = tel.state.pitch;
  bt.pitchRate = tel.state.pitchRate;
  bt.roll = tel.state.roll;
  bt.rollRate = tel.state.rollRate;
  bt.wheelVelLeft = wtel.left.velocity;
  bt.wheelVelRight = wtel.right.velocity;
  bt.bodyFwdVelocity = tel.state.forwardVelocity;
  bt.bodyYawRate = tel.state.turnVelocity;
  bt.targetFwdVelocity = tel.command.forwardVelocity;
  bt.targetYawRate = tel.command.turnVelocity;
  bt.wheelCmdLeft = tel.output.wheels.left.velocity;
  bt.wheelCmdRight = tel.output.wheels.right.velocity;

  _udp.beginPacket(kUdpAddress, kUdpPort);
  _udp.write((const uint8_t *)&bt, sizeof(bt));
  _udp.endPacket();
}