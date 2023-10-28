//Find offsets with bevel gears on the right side of the drive train
#define FL_WHEEL_OFFSET 0.474 // FB 0.11475   
#define FR_WHEEL_OFFSET 0.426 // FB 0.082
#define BR_WHEEL_OFFSET 0.865 // FB 0.841
#define BL_WHEEL_OFFSET 0.147 // FB 0.0312

//Assuming a rectangular drive train (input distance between center of wheels)
#define DRIVE_LENGTH 0.5906_m 
#define DRIVE_WIDTH 0.489_m 

//Encoder constants
#define DRIVE_MOTOR_GEAR_RATIO 7.36 
#define DRIVE_MOTOR_CIRCUMFERENCE 0.10469983 * M_PI
#define SPIN_MOTOR_GEAR_RATIO 15.43

// Swerve Module Wheel Spin PID Values
#define WHEEL_SPIN_KP 1
#define WHEEL_SPIN_KI 0
#define WHEEL_SPIN_KI_MAX 0.03
#define WHEEL_SPIN_KD 0
#define WHEEL_SPIN_ERROR 3
#define WHEEL_SPIN_MAX_SPEED 1.0