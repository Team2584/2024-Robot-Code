#ifndef INTAKE_CONSTANTS_H
#define INTAKE_CONSTANTS_H

// Ports for different motors and sensors (Spark MAX CAN ID - Check using REV firmware utility)
#define ON_WRIST_MOTOR_PORT 17
#define WRIST_MOTOR_PORT 18
#define MAIN_FIXED_INTAKE_MOTOR_PORT 19
#define SELECTOR_FIXED_INTAKE_MOTOR_PORT 20

#define INTAKE_IR_SENSOR_PORT 6
#define TUNNEL_IR_SENSOR_PORT 5

namespace IntakeConstants{
inline const double INTAKE_SPEED_IN = -50;
inline const double INTAKE_SPEED_OUT = 50;
inline const double SELECTOR_SPEED_ELEVATOR = 50;
inline const double SELECTOR_SPEED_SHOOTER = -50;

namespace Wrist{
inline const double WRIST_HIGH = 0.5;
inline const double WRIST_LOW = 0.72; //was 715
inline const double WRIST_SHOOT = 0.655;
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