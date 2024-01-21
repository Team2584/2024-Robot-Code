// Ports for different motors and sensors (Spark MAX CAN ID - Check using REV firmware utility)
#define INTAKE_MOTOR_PORT 17 
#define FIXED_INTAKE_MOTOR_PORT 18
#define WRIST_MOTOR_PORT 19

//Intake motor speeds, percent
#define INTAKE_SPEED_IN 1.0
#define INTAKE_SPEED_OUT 1.0

//Wrist absolute Encoder position values, 0-1 rotatons
#define WRIST_HIGH 0.700
#define WRIST_LOW 0.989

#define WRIST_SPEED_LOW_THRESHHOLD -2.0

//Wrist PID Constants
#define WRISTFF 0
#define WRISTKP 1
#define WRISTKI 0
#define WRISTKIMAX 0.1
#define ALLOWABLE_ERROR_WRIST 0.05
#define WRISTMAX_SPEED 0.3
