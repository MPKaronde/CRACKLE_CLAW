#include <Arduino.h>
#include "gripper.h"
#include "cli_control.h"

Gripper gripper(13, 14, 18, 19, 21, 22);

void setup() {
    CLI_Setup(&gripper);
}

void loop() {
    CLI(&gripper);
}
