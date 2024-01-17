// Ports for different motors and sensors (Spark MAX CAN ID - Check using REV firmware utility)
#define INTAKE_MOTOR_PORT 7 //????
#define FIXED_INTAKE_MOTOR_PORT 5
#define WRIST_MOTOR_PORT 21

//Intake motor speeds, percent
#define INTAKE_SPEED_IN 1.0
#define INTAKE_SPEED_OUT 1.0

//Wrist absolute Encoder position values, 0-1 rotatons
#define WRIST_HIGH 0.733
#define WRIST_LOW 0.999

#define WRIST_SPEED_LOW_THRESHHOLD -2.0

//Wrist PID Constants
#define WRISTFF 0
#define WRISTKP 2.0
#define WRISTKI 0
#define WRISTKIMAX 0.1
#define ALLOWABLE_ERROR_WRIST 0.05  
#define WRISTMAX_SPEED 0.7
