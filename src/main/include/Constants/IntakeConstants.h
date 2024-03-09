#ifndef INTAKE_CONSTANTS_H
#define INTAKE_CONSTANTS_H

#include "Robot.h"

// Ports for different motors and sensors (Spark MAX CAN ID - Check using REV firmware utility)
#define ON_WRIST_MOTOR_PORT 17
#define WRIST_MOTOR_PORT 18
#define MAIN_FIXED_INTAKE_MOTOR_PORT 19
#define SELECTOR_FIXED_INTAKE_MOTOR_PORT 20

#define INTAKE_IR_SENSOR_PORT 4
#define TUNNEL_IR_SENSOR_PORT 5

namespace IntakeConstants{
inline const double INTAKE_SPEED_IN = -0.9;
inline const double INTAKE_SPEED_OUT = 0.5;
inline const double FIXED_INTAKE_SPEED_IN = -0.5;
inline const double FIXED_INTAKE_SPEED_OUT = 0.7;
inline const double INTAKE_SPEED_BACK_TO_SELECTOR = 0.5;
inline const double SELECTOR_SPEED_ELEVATOR = 0.7;
inline const double SELECTOR_SPEED_SHOOTER = -0.95;
inline const double INTAKE_SPEED_SHOOTER = -0.9;
inline const units::second_t SHOT_INTAKE_TIME = 0.2_s;

namespace Wrist{
inline const double WRIST_HIGH = 0.195; ///0.538
inline const double WRIST_LOW = 0;
inline const double WRIST_SHOOT = 0.19;
inline constexpr auto KS = 0_V;
inline constexpr auto KG = 0.42_V;
inline constexpr auto KV = 0.4_V * 1_s / 1_rad; //0.88
inline double KD  = 0;
inline double KP  = 18;
inline double KI  = 0;
inline double KIMAX  = 0.1;
inline double POS_ERROR  = 0.015; //~3deg
inline double VELOCITY_ERROR = 1;
inline double MAX_SPEED  = 10;
inline double MIN_SPEED  = 0.05;
}
}

#endif