#ifndef CLI_CONTROL_H
#define CLI_CONTROL_H

#include "gripper.h"

/*
  Call once in setup() to initialize serial and zero the gripper.
*/
void CLI_Setup(Gripper* g);

/*
  Call every loop() iteration. Non-blocking — reads a line if one is available
  and dispatches to the appropriate Gripper method.

  Commands:
    grip              — close gripper until load cell contact, monitor for slip
    release           — open gripper to max angle
    zero servos       — move both servos to min angle
    zero loadcells    — tare load cells (remove all load first, takes ~3s)
    read              — print current load cell readings once
    monitor           — toggle continuous load cell printing
    servo <l|r> <0-180> — move one servo to a specific angle
    help              — print command list
*/
void CLI(Gripper* g);

#endif
