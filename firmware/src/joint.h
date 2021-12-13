#ifndef JOINT_H_
#define JOINT_H_

#include <Arduino.h>
#include "RampsStepper.h"
#include "endstop.h"
#include "robotGeometry.h"


class Joint {
    public:
    // Constructors
    Joint(int aStepPin, int aDirPin, int aEnablePin, bool aInverse, float main_gear_teeth, float motor_gear_teeth, int microsteps, int steps_per_rev, int a_min_pin, int a_min_input, int a_step_offset, int a_home_dwell, bool does_swap_pin, String name);
    Joint(RampsStepper &motor, Endstop &endstop);
    
    // MOTOR FUNCTIONS
    void setPositionRad();
    void stepToPositionRad(float value);
    void stepToPositionMM(float value, int steps_per_mm_rail);
    void updateMotor();
    void enableMotor(bool &enable);
    void oneStep();

    // ENDSTOP FUNCTIONS
    void setEndstopPins();
    int readEndstopPin();
    void homeOffset();
    
    // private:
    String name;
    bool aInverse;
    float orig_angle;

    RampsStepper motor;
    Endstop endstop;
    bool inverse_stepper;


};

#endif
