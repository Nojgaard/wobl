/**
 * Wheel hardware test — self-running, no host tooling required.
 *
 * Uses the real wheelTaskInit / wheelTask production code path.
 * Commands go through SharedState; telemetry is read back the same way.
 *
 * Upload:  pio run -e wheel_test -t upload
 * Monitor: pio device monitor -b 115200
 *
 * Verifies fundamentals only: init, sensor alive, direction sign.
 * PID tuning / step-response characterisation deferred to host API.
 */

#include "wheel_task.hpp"
#include "shared_state.hpp"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static constexpr float kTestVelocity  = 1.0f;  // rad/s — safe bench speed
static constexpr long  kRunMs         = 5000;   // ms per direction
static constexpr long  kSampleMs      = 200;    // ms between samples
// Expected angle_delta at kTestVelocity over kRunMs: 1.0 * 5.0 = 5.0 rad.
// Require at least 50% of that — generous to allow for PID undershoot.
static constexpr float kMinAngleDelta = 2.5f;   // rad
// Direction check: angle_delta sign must match command sign.
// avg_vel is kept for diagnostics only — LPF lag makes it unreliable for PASS/FAIL.

static SharedState state;

// ── Test helpers ─────────────────────────────────────────────────────────────

struct StepResult {
    float avgVelocity;
    float angleDelta;
};

// Run one velocity step and collect telemetry via SharedState.
// side: 0 = left, 1 = right
static StepResult runStep(int side, float target, long durationMs) {
    WheelsCommand cmd = state.commands.wheels.read();
    if (side == 0) { cmd.left  = {.enabled = true, .velocity = target}; }
    else           { cmd.right = {.enabled = true, .velocity = target}; }
    state.commands.wheels.write(cmd);

    float velSum     = 0.0f;
    int   velSamples = 0;
    float angleStart = (side == 0)
        ? state.telemetry.wheels.read().left.angle
        : state.telemetry.wheels.read().right.angle;

    long next = millis() + kSampleMs;
    long end  = millis() + durationMs;
    while (millis() < end) {
        if (millis() >= next) {
            WheelsData telem = state.telemetry.wheels.read();
            velSum += (side == 0) ? telem.left.velocity : telem.right.velocity;
            velSamples++;
            next += kSampleMs;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // Disable the tested side
    cmd = state.commands.wheels.read();
    if (side == 0) { cmd.left  = {.enabled = false, .velocity = 0.0f}; }
    else           { cmd.right = {.enabled = false, .velocity = 0.0f}; }
    state.commands.wheels.write(cmd);

    // Short coast before reading final angle
    vTaskDelay(300 / portTICK_PERIOD_MS);

    float angleEnd = (side == 0)
        ? state.telemetry.wheels.read().left.angle
        : state.telemetry.wheels.read().right.angle;

    return {
        .avgVelocity = velSamples > 0 ? velSum / velSamples : 0.0f,
        .angleDelta  = angleEnd - angleStart,
    };
}

static void testWheel(const char *name, int side) {
    Serial.printf("\n── %s ──────────────────────────\n", name);

    // ── CHECK 1: init ────────────────────────────────────────────────────
    WheelsStatus ws = state.status.wheels.read();
    bool initOk = (side == 0) ? (ws.leftStatus == 1) : (ws.rightStatus == 1);
    Serial.printf("[%s] INIT     %s  (status=%d)\n",
                  name, initOk ? "PASS" : "FAIL",
                  side == 0 ? ws.leftStatus : ws.rightStatus);
    if (!initOk) {
        Serial.printf("[%s] Skipping motion tests.\n", name);
        return;
    }

    // ── CHECK 2 & 3: positive direction ─────────────────────────────────
    Serial.printf("[%s] Running +%.1f rad/s for %ld ms...\n", name, kTestVelocity, kRunMs);
    StepResult pos = runStep(side, +kTestVelocity, kRunMs);
    float posEffVel = pos.angleDelta / (kRunMs / 1000.0f); // avg from angle — LPF-independent
    Serial.printf("[%s]   eff_vel=%.2f rad/s  angle_delta=%.2f rad  (lpf_vel=%.2f)\n",
                  name, posEffVel, pos.angleDelta, pos.avgVelocity);

    bool posAngle = fabsf(pos.angleDelta) > kMinAngleDelta;
    bool posSign  = pos.angleDelta > 0.0f;
    Serial.printf("[%s] SENSOR+  %s  (|angle_delta|=%.2f, need >%.1f)\n",
                  name, posAngle ? "PASS" : "FAIL", fabsf(pos.angleDelta), kMinAngleDelta);
    Serial.printf("[%s] DIR+     %s  (angle_delta=%.2f, expect >0)\n",
                  name, posSign ? "PASS" : "FAIL", pos.angleDelta);

    // ── CHECK 4: negative direction ──────────────────────────────────────
    Serial.printf("[%s] Running -%.1f rad/s for %ld ms...\n", name, kTestVelocity, kRunMs);
    StepResult neg = runStep(side, -kTestVelocity, kRunMs);
    float negEffVel = neg.angleDelta / (kRunMs / 1000.0f);
    Serial.printf("[%s]   eff_vel=%.2f rad/s  angle_delta=%.2f rad  (lpf_vel=%.2f)\n",
                  name, negEffVel, neg.angleDelta, neg.avgVelocity);

    bool negAngle = fabsf(neg.angleDelta) > kMinAngleDelta;
    bool negSign  = neg.angleDelta < 0.0f;
    Serial.printf("[%s] SENSOR-  %s  (|angle_delta|=%.2f, need >%.1f)\n",
                  name, negAngle ? "PASS" : "FAIL", fabsf(neg.angleDelta), kMinAngleDelta);
    Serial.printf("[%s] DIR-     %s  (angle_delta=%.2f, expect <0)\n",
                  name, negSign ? "PASS" : "FAIL", neg.angleDelta);

    bool allPass = posAngle && posSign && negAngle && negSign;
    Serial.printf("[%s] ══ %s ══\n", name, allPass ? "ALL PASS" : "SOME CHECKS FAILED");
}

// ── Test task (core 0) ────────────────────────────────────────────────────────

static void testTask(void *) {
    // Give wheelTaskInit + first FOC cycles time to settle
    vTaskDelay(500 / portTICK_PERIOD_MS);

    WheelsStatus ws = state.status.wheels.read();
    Serial.printf("[foc] loop rate: %.0f Hz\n", ws.focRate);
    Serial.printf("[foc] SharedState sync rate: %.0f Hz\n", ws.updateRate);

    testWheel("LEFT",  0);
    testWheel("RIGHT", 1);

    Serial.println("\n=== Test complete. Safe to power off. ===");
    vTaskDelete(NULL);
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
    WiFi.mode(WIFI_OFF);
    esp_bt_controller_disable();

    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Wheel hardware test ===");

    Serial.println("Send any key to start tests...");
    while (!Serial.available()) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    while (Serial.available()) Serial.read(); // flush
    Serial.println("Starting...");

    wheelTaskInit(state);

    // FOC loop — same core and priority as production
    xTaskCreatePinnedToCore(wheelTask, "foc",  4096, &state, 24, NULL, 1);
    // Test sequencer — core 0, lower priority
    xTaskCreatePinnedToCore(testTask,  "test", 4096, NULL,    5, NULL, 0);
}

void loop() { vTaskDelete(NULL); }
