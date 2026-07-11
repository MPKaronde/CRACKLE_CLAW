#include <Arduino.h>
#include "gripper.h"

// Servo pins (left, right), then HX711 (dt, sck) for each load cell.
Gripper gripper(13, 14, 18, 19, 21, 22);

// Serial protocol (line based, 115200 baud) — mirrored by the ROS driver
// in claw_degree_publisher (see crackle_claw.py):
//   incoming:  '1' -> close (release then grip_object)
//              '0' -> open  (release)
//   outgoing:  "ANGLE <deg>\n" every REPORT_INTERVAL_MS with the current
//              (symmetric) servo angle, so ROS can publish joint state.
static const unsigned long REPORT_INTERVAL_MS = 50;

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing CRACKLE Claw...");
    gripper.zero_servos();
    // Zero load cells with no load applied.
    gripper.zero_loadcells();
    Serial.println("Ready");
}

void loop() {
    // --- Command handling: single-char open/close, keeps /claw/command interface ---
    while (Serial.available()) {
        int c = Serial.read();
        if (c == '1') {
            // Close: open fully first for a consistent starting point, then grip.
            gripper.release();
            gripper.grip_object();
        } else if (c == '0') {
            // Open.
            gripper.release();
        }
        // Any other byte (whitespace, newline, stray input) is ignored.
    }

    // --- Continuous angle report so ROS can publish joint positions ---
    static unsigned long last_report = 0;
    unsigned long now = millis();
    if (now - last_report >= REPORT_INTERVAL_MS) {
        last_report = now;
        Serial.printf("ANGLE %d\n", gripper.get_angle(LEFT));
    }
}
