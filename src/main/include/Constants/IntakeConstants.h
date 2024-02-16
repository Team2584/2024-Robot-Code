#ifndef INTAKE_CONSTANTS_H
#define INTAKE_CONSTANTS_H

// Ports for different motors and sensors (Spark MAX CAN ID - Check using REV firmware utility)
#define ON_WRIST_MOTOR_PORT 17
#define WRIST_MOTOR_PORT 18
#define MAIN_FIXED_INTAKE_MOTOR_PORT 19
#define SELECTOR_FIXED_INTAKE_MOTOR_PORT 20

#define INTAKE_IR_SENSOR_PORT 4
#define TUNNEL_IR_SENSOR_PORT 5

namespace IntakeConstants{
inline const double INTAKE_SPEED_IN = 1;
inline const double INTAKE_SPEED_OUT = 0.5;
inline const double FIXED_INTAKE_SPEED_IN = -0.7;
inline const double FIXED_INTAKE_SPEED_OUT = 0.7;
inline const double INTAKE_SPEED_BACK_TO_SELECTOR = 0.5;
inline const double SELECTOR_SPEED_ELEVATOR = 0.7;
inline const double SELECTOR_SPEED_SHOOTER = -0.7;

namespace Wrist{
inline const double WRIST_HIGH = 0.22;
inline const double WRIST_LOW = 0.0;
inline const double WRIST_SHOOT = 0.17;
inline double KD  = 0;
inline double KP  = 4;
inline double KI  = 0.03;
inline double KIMAX  = 0.1;
inline double POS_ERROR  = 0.015; //~3deg
inline double VELOCITY_ERROR = 1;
inline double MAX_SPEED  = 0.3;
inline double MIN_SPEED  = 0.05;
}
}

#endif