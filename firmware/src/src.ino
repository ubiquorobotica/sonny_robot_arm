//20SFFACTORY COMMUNITY ROBOT FIRMWARE

//MAINTAINER: LEOYEUNG@20SFFACTORY
//CONTACT: yeung.kl.leo@gmail.com
//FORUM: www.facebook.com/groups/robotarm
//DOCUMENTATION: www.20sffactory.com/robot/resource

//VERSION: V0.71

//VERSION HISTORY:
//   V0.31 WITH G92, M114, LOGGER, LIMIT_CHECK FUNCTIONS
//   V0.41 WITH DUAL SHANK LENGTH SUPPORT
//   V0.51 WITH SERVO GRIPPER
//   V0.61 WITH ARDUINO UNO OPTION
//   V0.71 WITH:
//       ESP32(WEMOS D1R32) WITH PS4 JOYSTICK CONTROL OPTION
//       COMMAND TO SET CUSTOM SPEED PROFILE 'M205 S0'
//       UNO OPTION WITH RAIL SUPPORT

#include <Arduino.h>
//GENERAL CONFIG SETTINGS
#include "config.h"

#include "robotGeometry.h"
#include "interpolation.h"
#include "RampsStepper.h"
#include "queue.h"
#include "command.h"
#include "equipment.h"
#include "endstop.h"
#include "logger.h"
#include "fanControl.h"
#include "joint.h"
 #include "robot.h"
//INCLUDE CORRESPONDING GRIPPER MOTOR CLASS
#if GRIPPER == SERVO
  #include "servo_gripper.h"
#elif GRIPPER == BYJ
  #include "byj_gripper.h"
#endif

#include "pinout/pinout.h"

//RAIL OBJECTS
#if RAIL
  RampsStepper stepperRail(E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, INVERSE_E0_STEPPER, MAIN_GEAR_TEETH, MOTOR_GEAR_TEETH, MICROSTEPS, STEPS_PER_REV);
#endif

// Group stepper and endpoints into joints
Joint J1(Z_MIN_PIN, Z_DIR_PIN, Z_STEP_PIN, Z_ENABLE_PIN, INVERSE_Z_STEPPER, J1_GEAR_TEEH, MOTOR_GEAR_TEETH, MICROSTEPS, STEPS_PER_REV, Z_MIN_INPUT, Z_HOME_STEPS, HOME_DWELL, false, "J1");
Joint J2(Y_MIN_PIN, Y_DIR_PIN, Y_STEP_PIN, Y_ENABLE_PIN, INVERSE_Y_STEPPER, J2_J3_GEAR_TEETH, MOTOR_GEAR_TEETH, MICROSTEPS, STEPS_PER_REV, Y_MIN_INPUT, Y_HOME_STEPS, HOME_DWELL, false, "J2");
Joint J3(X_MIN_PIN, X_DIR_PIN, X_STEP_PIN, X_ENABLE_PIN, INVERSE_X_STEPPER, J2_J3_GEAR_TEETH, MOTOR_GEAR_TEETH, MICROSTEPS, STEPS_PER_REV, X_MIN_INPUT, X_HOME_STEPS, HOME_DWELL, false, "J3");

//RAIL OBJECTS
#if RAIL
  RampsStepper stepperRail(E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, INVERSE_E0_STEPPER, MAIN_GEAR_TEETH, MOTOR_GEAR_TEETH, MICROSTEPS, STEPS_PER_REV);
  #if BOARD_CHOICE == WEMOSD1R32 //PINSWAP REQIURED ON D1R32 DUE TO INSUFFICIENT DIGIAL PINS
    Endstop endstopE0(E0_MIN_PIN, E0_DIR_PIN, E0_STEP_PIN, E0_ENABLE_PIN, E0_MIN_INPUT, E0_HOME_STEPS, HOME_DWELL, true);
  #else 
    Endstop endstopE0(E0_MIN_PIN, E0_DIR_PIN, E0_STEP_PIN, E0_ENABLE_PIN, E0_MIN_INPUT, E0_HOME_STEPS, HOME_DWELL, false);
  #endif
  // Joint rail(E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, INVERSE_E0_STEPPER, MAIN_GEAR_TEETH, MOTOR_GEAR_TEETH, MICROSTEPS, STEPS_PER_REV, E0_MIN_PIN, E0_MIN_INPUT, E0_HOME_STEPS, HOME_DWELL, true)
  Joint rail(stepperRail, endstopE0)
#endif

//EQUIPMENT OBJECTS
#if GRIPPER == SERVO
  Servo_Gripper servo_gripper(SERVO_PIN, SERVO_GRIP_DEGREE, SERVO_UNGRIP_DEGREE);
#elif GRIPPER == BYJ
  BYJ_Gripper byj_gripper(BYJ_PIN_0, BYJ_PIN_1, BYJ_PIN_2, BYJ_PIN_3, BYJ_GRIP_STEPS);
#endif
Equipment laser(LASER_PIN);
Equipment pump(PUMP_PIN);
Equipment led(LED_PIN);
FanControl fan(FAN_PIN, FAN_DELAY);

RobotGeometry geometry(END_EFFECTOR_OFFSET, LOW_SHANK_LENGTH, HIGH_SHANK_LENGTH);
Interpolation interpolator;
Robot robot(interpolator, geometry, J1, J2, J3, fan);

// COMMAND OBJECTS 
Queue<Cmd> queue(QUEUE_SIZE);
Command command;

//PS4 CONTROLLER OBJECT FOR ESP32
#if BOARD_CHOICE == WEMOSD1R32 && ESP32_PS4_CONTROLLER
  #include "controller_ps4.h"
  Controller_PS4 controller_ps4(PS4_MAC);
#endif

void setup()
{
  Serial.begin(BAUD);
  
  #if BOARD_CHOICE == WEMOSD1R32 && ESP32_PS4_CONTROLLER
    controller_ps4.setup();
  #endif

  robot.setPositionRad();

  if (HOME_ON_BOOT) { //HOME DURING SETUP() IF HOME_ON_BOOT ENABLED
    // homeSequence();
    robot.homeSequence();
    Logger::logINFO("ROBOT ONLINE");
  } else {
    robot.setStepperEnable(false); //ROBOT ADJUSTABLE BY HAND AFTER TURNING ON
    if (HOME_X_STEPPER && HOME_Y_STEPPER && !HOME_Z_STEPPER){
      Logger::logINFO("ROBOT ONLINE");
      Logger::logINFO("ROTATE ROBOT TO FACE FRONT CENTRE & SEND G28 TO CALIBRATE");
    }
    if (HOME_X_STEPPER && HOME_Y_STEPPER && HOME_Z_STEPPER){
      Logger::logINFO("ROBOT ONLINE");
      Logger::logINFO("SEND G28 TO CALIBRATE");
    }
    if (!HOME_X_STEPPER && !HOME_Y_STEPPER){
      Logger::logINFO("ROBOT ONLINE");
      Logger::logINFO("HOME ROBOT MANUALLY & SEND G28 TO CALIBRATE");
    }
  }
  // interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
  robot.interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
}

void loop() {
  robot.update();
  
  if (!queue.isFull()) {
    if (command.handleGcode()) {
      queue.push(command.getCmd());
    }
  }
  if ((!queue.isEmpty()) && interpolator.isFinished()) {
    executeCommand(queue.pop());
    if (PRINT_REPLY) {
      Serial.println(PRINT_REPLY_MSG);
    }
  }

  if (millis() % 500 < 250) {
    led.cmdOn();
  }
  else {
    led.cmdOff();
  }

  #if BOARD_CHOICE == WEMOSD1R32 && ESP32_PS4_CONTROLLER
    ps4_controller_loop();
  #endif
}

void executeCommand(Cmd cmd) {

  if (cmd.id == -1) {
    printErr();
    return;
  }

  if (cmd.id == 'G') {
    switch (cmd.num) {
    case 0:
    case 1:
      fan.enable(true);
      Point posoffset;
      posoffset = interpolator.getPosOffset();      
      cmdMove(cmd, interpolator.getPosmm(), posoffset, command.isRelativeCoord);
      interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE, cmd.valueF);
      Logger::logINFO("LINEAR MOVE: [X:" + String(cmd.valueX-posoffset.xmm) + " Y:" + String(cmd.valueY-posoffset.ymm) + " Z:" + String(cmd.valueZ-posoffset.zmm) + " E:" + String(cmd.valueE-posoffset.emm)+"]");
      break;
    case 4: cmdDwell(cmd); break;
    case 28: 
      robot.homeSequence();
      break;
    case 90: command.cmdToAbsolute(); break; // ABSOLUTE COORDINATE MODE
    case 91: command.cmdToRelative(); break; // RELATIVE COORDINATE MODE
    case 92: 
      interpolator.resetPosOffset();
      cmdMove(cmd, interpolator.getPosmm(), interpolator.getPosOffset(), false);
      interpolator.setPosOffset(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE);
      break;
    default: printErr();
    }
  }
  else if (cmd.id == 'M') {
    switch (cmd.num) {
    case 1: pump.cmdOn(); break;
    case 2: pump.cmdOff(); break;
    case 3: 
      #if GRIPPER == BYJ
        byj_gripper.cmdOn(); break;
      #elif GRIPPER == SERVO
        servo_gripper.cmdOn(); break;
      #endif
    case 5:
      #if GRIPPER == BYJ
        byj_gripper.cmdOff(); break;
      #elif GRIPPER == SERVO
        servo_gripper.cmdOff(); break;
      #endif
    case 6: laser.cmdOn(); break;
    case 7: laser.cmdOff(); break;
    case 17: robot.setStepperEnable(true); break;
    case 18: robot.setStepperEnable(false); break;
    case 106: fan.enable(true); break;
    case 107: fan.enable(false); break;
    case 114: command.cmdGetPosition(interpolator.getPosmm(), interpolator.getPosOffset(), J3.motor.getPosition(), J2.motor.getPosition(), J1.motor.getPosition()); break;// Return the current positions of all axis 
    case 119: {
      String endstopMsg = "ENDSTOP: [X:";
      endstopMsg += String(J3.endstop.state());
      endstopMsg += " Y:";
      endstopMsg += String(J2.endstop.state());
      endstopMsg += " Z:";
      endstopMsg += String(J1.endstop.state());
      #if RAIL
        endstopMsg += " E:";
        endstopMsg += String(endstopE0.state());
      #endif
      endstopMsg += "]";
      //ORIGINAL LOG STRING UNDESIRABLE FOR UNO PROCESSING
      //Logger::logINFO("ENDSTOP STATE: [UPPER_SHANK(X):"+String(J3.endstop.state())+" LOWER_SHANK(Y):"+String(J2.endstop.state())+" ROTATE_GEAR(Z):"+String(J1.endstop.state())+"]");
      Logger::logINFO(endstopMsg);
      break;}
    case 205:
      interpolator.setSpeedProfile(cmd.valueS); 
      Logger::logINFO("SPEED PROFILE: [" + String(interpolator.speed_profile) + "]");
      break;
    default: printErr();
    }
  }
  else {
    printErr();
  }
}

//void homeSequence(){
////  setStepperEnable(false);  
//  fan.enable(true);
//  Logger::logINFO("HOMING..");
//  delay(2000);
//  if (HOME_Y_STEPPER && HOME_X_STEPPER && HOME_Z_STEPPER){
//
//    int x_endstop_state, y_endstop_state, z_endstop_state;
//    x_endstop_state = digitalRead(J3.endstop.getMinPin());
//    y_endstop_state = digitalRead(J2.endstop.getMinPin());
//    z_endstop_state = digitalRead(J1.endstop.getMinPin());
//
//    J3.endstop.setPins(!INVERSE_X_STEPPER);
//    J2.endstop.setPins(!INVERSE_Y_STEPPER);
//    J1.endstop.setPins(!INVERSE_Z_STEPPER);
//    
//    delayMicroseconds(5);
//    while (!x_endstop_state || !y_endstop_state || !z_endstop_state) {
//      if (!x_endstop_state) {
//        digitalWrite(J3.endstop.getStepPin(), HIGH);
//        digitalWrite(J3.endstop.getStepPin(), LOW);
//        x_endstop_state = digitalRead(J3.endstop.getMinPin());
//      }
//      if (!y_endstop_state) {
//        digitalWrite(J2.endstop.getStepPin(), HIGH);
//        digitalWrite(J2.endstop.getStepPin(), LOW);
//        y_endstop_state = digitalRead(J2.endstop.getMinPin());
//      }
//      if (!z_endstop_state) {
//        digitalWrite(J1.endstop.getStepPin(), HIGH);
//        digitalWrite(J1.endstop.getStepPin(), LOW);
//        z_endstop_state = digitalRead(J1.endstop.getMinPin());
//      }
//      delayMicroseconds(HOME_DWELL);
//      }
//
//    delay(700);  
//    J3.endstop.homeOffset(!INVERSE_X_STEPPER);
//    J2.endstop.homeOffset(!INVERSE_Y_STEPPER);
//    J1.endstop.homeOffset(!INVERSE_Z_STEPPER);
//
//   // Don't know what this is
////   if (swap_pin == true){
////     pinMode(min_pin, OUTPUT);
////     delayMicroseconds(5);
////   }
//  }
//  #if RAIL
//    if (HOME_E0_STEPPER){
//      endstopE0.home(!INVERSE_E0_STEPPER);
//    }
//  #endif
//  interpolator.setInterpolation(INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0, INITIAL_X, INITIAL_Y, INITIAL_Z, INITIAL_E0);
//  Logger::logINFO("HOMING COMPLETE");
//}
