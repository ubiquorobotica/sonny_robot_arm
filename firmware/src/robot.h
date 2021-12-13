#ifndef ROBOT_H_
#define ROBOT_H_

#include <Arduino.h>
#include "interpolation.h"
#include "robotGeometry.h"
#include "joint.h"
#include "fanControl.h"

class Robot {
    public:
    Robot(Interpolation &interpolation, RobotGeometry &geometry, Joint &J1, Joint &J2, Joint &J3, FanControl &fan);
    Robot(Interpolation &interpolation, RobotGeometry &geometry, Joint &J1, Joint &J2, Joint &J3, FanControl &fan, Joint &rail);

    Interpolation &interpolator;
    RobotGeometry &geometry;
    Joint &J1;
    Joint &J2;
    Joint &J3;
    Joint &_rail;
    FanControl &fan;

    void update();
    void setPositionRad();
    void homeSequence();
    void setStepperEnable(bool enable);
    void homeOffset();

    private:
    bool has_rail;
};

#endif
