#ifndef GRIPPER_H
#define GRIPPER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <HX711.h>

enum Side
{
    LEFT, RIGHT
};

class Gripper
{
public:
    Gripper(int left_servo, int right_servo, int left_dt, 
        int left_scl, int right_dt, int right_scl);

    /*
    Moves all servos to min_angle
    */
    void zero_servos();

    /*
    Note: to be called when loadcells are under no pressure
    Let load cells sit with no load for ~3 seconds.
    Finds average unloaded reading and sets load_cell offsets accordingly
    */
    void zero_loadcells();

    /*
    Returns load cell reading for given side
    */
    float get_loadcell_reading(Side side);

    /*
    Move servo on given side to given angle
    */
    void move_servo(Side side, int angle);

    /*
    Returns the last commanded angle (degrees) for the given side.
    Both servos are driven symmetrically, so either side reflects the
    current jaw position. Used by the ROS driver to publish joint state.
    */
    int get_angle(Side side) const;

    /*
    Closes servos until loadcell readings spike
    If loadcell readings appear to lower (indicating slip), will increase servo force
    */
    void grip_object();

    /*
    Moves servos to all the way out (max) position
    */
    void release();

private:
    const int LEFT_SERVO_PIN;       // left servo pin
    const int RIGHT_SERVO_PIN;      // right servo pin
    const int DT_LEFT;              // DT for the left load cell
    const int SCL_LEFT;             // SCL for left load cell
    const int DT_RIGHT;             // DT for right load cell
    const int SCL_RIGHT;            // SCL for right load cell

    float left_cell_offset;         // subtracts from readings
    float right_cell_offset;        // subtracts from readings

    Servo left_servo;
    Servo right_servo;
    HX711 left_cell;
    HX711 right_cell;

    int left_angle;                 // tracked current angle
    int right_angle;

};

#endif