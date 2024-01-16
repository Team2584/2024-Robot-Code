// Ports for different motors and sensors (Spark MAX CAN ID - Check using REV firmware utility)
#define INTAKE_MOTOR_PORT 7 //????
#define WRIST_MOTOR_PORT 21

//Intake motor speeds, percent
#define INTAKE_SPEED_IN 1
#define INTAKE_SPEED_OUT 1

//Wrist absolute Encoder position values, 0-1 rotatons
#define WRIST_HIGH 0.775
#define WRIST_LOW 0.995

#define WRIST_SPEED_LOW_THRESHHOLD -0.2

//Wrist PID Constants
#define WRISTFF 0
#define WRISTKP 0.4
#define WRISTKI 0
#define WRISTKIMAX 0.1
#define ALLOWABLE_ERROR_WRIST 0.05  
#define WRISTMAX_SPEED 0.5
