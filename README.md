# Sonny Robot Arm

The Sonny Robot Arm is an open-source project dedicated to fostering education in robotics. It is a 4-axis desktop robot arm that balances cost-effectiveness with high-quality performance. The project's structure and documentation aim to provide a solid foundation for learning and experimentation in the field of robotics.

![Sonny Robot Arm](images/sonny_1.jpg)

## Table of Contents
- [Repository Structure](#repository-structure)
- [Project Goals](#project-goals)
- [Key Features](#key-features)
- [Hardware](#hardware)
- [Software](#software)
- [Documentation](#documentation)
- [License](#license)
- [Contributing](#contributing)
- [Future Developments](#future-developments)

## **Repository Structure**

The repository is organized into several Git submodules:

- **Sonny Firmware**: C++ firmware for the robot arm.
- **Sonny Design Files**: CAD files and hardware specifications.
- **UROCS**: Cross-platform GUI for robot control.
- **Documentation**: Guides and references for users.

## **Project Goals**

The creation of the Sonny Robot Arm is driven by two main goals:

- **Educational Value**: To serve as a practical tool for learning about robotics and automation.
- **Quality at Low Cost**: To deliver a robot arm that is affordable without compromising on its capabilities.

## **Features**

- **4-Axis Design**: Based on the ABB IRB 460 kinematics for precise movement control.
- **Educational Tool**: Ideal for learning about robotics, kinematics, and control systems.
- **Payload**: Capable of handling payloads up to 500 grams.
- **Durability**: Equipped with robust bearings in all axis for smooth operation.
- **Motors**: NEMA 17 stepper motors and an MG90S servo motor for precision.
- **Transmission**: Belt-driven with reducers to reduce joint play.
- **Microcontroller**: Arduino MEGA 2560 with RAMPS 1.4 board.
- **Calibration**: Includes limit switches for homing sequences.

## **Hardware Specifications**

The robot arm utilizes a combination of NEMA 17 stepper motors, an MG90S servo motor, belt-driven transmission, and an Arduino-based control system to meet its design objectives.

## **Software Overview**

The Sonny Robot Arm operates using:

- **Firmware**: Handles motion planning and communication.
- **UROCS**: Electron.js-based GUI for robot interaction and monitoring.

## **Documentation**

Detailed documentation is provided to guide users through assembly, software setup, and operation of the robot arm.

## **License**

The project is open-source under a non-commercial license, encouraging use, modification, and sharing within the community while respecting the terms of non-commercial usage.

## **Contribution Guidelines**

Contributions are welcomed across various aspects of the project, including code development, documentation, hardware design, and testing. Please refer to the project's contribution guidelines for detailed instructions.

## **Future Roadmap**

Planned enhancements include URDF integration, a custom ESP32-based control board, ROS 2 integration, computer vision capabilities, and exploration of reinforcement learning algorithms.

## **Closing Note**

This project represents a journey over two years, balancing professional and academic commitments. It's released with the hope that others will build upon it, advancing both their understanding and the field of robotics.
