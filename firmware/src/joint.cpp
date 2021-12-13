#include "joint.h"
#include <Arduino.h>


Joint::Joint(int aStepPin, int aDirPin, int aEnablePin, bool aInverse, float main_gear_teeth, float motor_gear_teeth, int microsteps, int steps_per_rev, int a_min_pin, int a_min_input, int a_step_offset, int a_home_dwell, bool does_swap_pin, String name)
: motor(aStepPin, aDirPin, aEnablePin, aInverse, main_gear_teeth, motor_gear_teeth, microsteps, steps_per_rev), endstop(a_min_pin, aDirPin, aStepPin, aEnablePin, aInverse, a_step_offset, a_home_dwell, does_swap_pin) {
    name = name;
    aInverse = aInverse;
    if (name == "J3") {
        orig_angle = PI/2;
    } else {
        orig_angle = 0.0;
    }   
}

Joint::Joint(RampsStepper &motor, Endstop &endstop) : motor(motor), endstop(endstop) {}

void Joint::setPositionRad() {
    motor.setPositionRad(orig_angle);
}

void Joint::stepToPositionRad(float value) {
    motor.stepToPositionRad(value);
}

void Joint::stepToPositionMM(float value, int steps_per_mm_rail) {
    motor.stepToPositionMM(value, steps_per_mm_rail);
}

void Joint::updateMotor() {
    motor.update();
}

void Joint::oneStep() {
    digitalWrite(endstop.getStepPin(), HIGH);
    digitalWrite(endstop.getStepPin(), LOW);
}

void Joint::homeOffset() {
    endstop.homeOffset(aInverse);
}

void Joint::enableMotor(bool &enable) {
    motor.enable(enable);
}

int Joint::readEndstopPin() {
    return digitalRead(endstop.getStepPin()); 
}

void Joint::setEndstopPins() {
    endstop.setPins(!aInverse);
}
