#include "robot.h"
#include "config.h"
#include "logger.h"

Robot::Robot(Interpolation &interpolator, RobotGeometry &geometry, Joint &J1, Joint &J2, Joint &J3, FanControl &fan)
: interpolator(interpolator), geometry(geometry), J1(J1), J2(J2), J3(J3), fan(fan) {
  has_rail = false;
  bool _rail = false;
};

Robot::Robot(Interpolation &interpolator, RobotGeometry &geometry, Joint &J1, Joint &J2, Joint &J3, FanControl &fan, Joint &rail)
: interpolator(interpolator), geometry(geometry), J1(J1), J2(J2), J3(J3), fan(fan), _rail(rail){
  has_rail = true;
};

void Robot::setPositionRad() {
  J3.setPositionRad(); // 90°
  J2.setPositionRad();         // 0°
  J1.setPositionRad();        // 0°
  if (this->has_rail) {
    _rail.setPositionRad();
  }
};

void Robot::update() {
  
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());

  J1.stepToPositionRad(geometry.getRotRad());
  J2.stepToPositionRad(geometry.getLowRad());
  J3.stepToPositionRad(geometry.getHighRad());
  if (this->has_rail) {
    _rail.stepToPositionMM(interpolator.getEPosmm(), STEPS_PER_MM_RAIL);
  }

  J1.updateMotor();
  J2.updateMotor();
  J3.updateMotor();
  if (this->has_rail) {
    _rail.updateMotor();
  }

  fan.update();
};

void Robot::setStepperEnable(bool enable) {
  J1.enableMotor(enable);
  J2.enableMotor(enable);
  J3.enableMotor(enable);

  if (this->has_rail) {
    _rail.enableMotor(enable);
  }
  fan.enable(enable);
}

void Robot::homeOffset() {
  J3.homeOffset();
  J2.homeOffset();
  J1.homeOffset();
  if (this->has_rail) {
    _rail.homeOffset();
  }
}

void Robot::homeSequence() {
  setStepperEnable(false); 
  fan.enable(true);

  Logger::logINFO("HOMING..");
  delay(2000);
  
  if (HOME_Y_STEPPER && HOME_X_STEPPER && HOME_Z_STEPPER){

    int j1_endstop_state, j2_endstop_state, j3_endstop_state;
    j1_endstop_state = J1.readEndstopPin();
    j2_endstop_state = J2.readEndstopPin();
    j3_endstop_state = J3.readEndstopPin();

    J3.setEndstopPins();
    J2.setEndstopPins();
    J1.setEndstopPins();
    
    delayMicroseconds(5);
    while (!j1_endstop_state || !j2_endstop_state || !j3_endstop_state) {
      if (!j1_endstop_state) {
        J1.oneStep();
        j1_endstop_state = J1.readEndstopPin();
      }
      if (!j2_endstop_state) {
        J2.oneStep();
        j2_endstop_state = J2.readEndstopPin();
      }
      if (!j3_endstop_state) {
        J3.oneStep();
        j3_endstop_state = J3.readEndstopPin();
      }
      delayMicroseconds(HOME_DWELL);
      }

    delay(700);  
  }   
  // Don't know what this is
//   if (swap_pin == true){
//     pinMode(min_pin, OUTPUT);
//     delayMicroseconds(5);
//   }

  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
  Logger::logINFO("HOMING COMPLETE");
}
