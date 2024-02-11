#ifndef INTAKE_CONSTANTS_H
#define INTAKE_CONSTANTS_H

// Ports for different motors and sensors (Spark MAX CAN ID - Check using REV firmware utility)
#define ON_WRIST_MOTOR_PORT 17
#define WRIST_MOTOR_PORT 18
#define FIXED_INTAKE_MOTOR_PORT 19
#define FIXED_INTAKE_MOTOR_PORT_2 20


//Intake motor speeds, percent
#define INTAKE_SPEED_IN 100
#define INTAKE_SPEED_OUT 100

namespace IntakeWrist{
    //Wrist absolute Encoder position values, 0-1 rotatons
    inline const double WRIST_HIGH = 0.5;
    inline const double WRIST_LOW = 0.72; //was 715
    inline const double WRIST_SHOOT = 0.655;
}

//Wrist PID Constants
inline double WRISTKD  = 0;
inline double WRISTKP  = 4;
inline double WRISTKI  = 0.03;
inline double WRISTKIMAX  = 0.1;
inline double WRIST_POS_ERROR  = 0.015; //~3deg
inline double WRIST_VELOCITY_ERROR = 1;
inline double WRISTMAX_SPEED  = 0.3;
inline double WRISTMIN_SPEED  = 0.05;

#endif