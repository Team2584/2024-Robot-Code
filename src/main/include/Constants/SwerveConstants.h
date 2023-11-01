/*
   ____                           _    ____                _              _       
  / ___| ___ _ __   ___ _ __ __ _| |  / ___|___  _ __  ___| |_ __ _ _ __ | |_ ___ 
 | |  _ / _ \ '_ \ / _ \ '__/ _` | | | |   / _ \| '_ \/ __| __/ _` | '_ \| __/ __|
 | |_| |  __/ | | |  __/ | | (_| | | | |__| (_) | | | \__ \ || (_| | | | | |_\__ \
  \____|\___|_| |_|\___|_|  \__,_|_|  \____\___/|_| |_|___/\__\__,_|_| |_|\__|___/
                                                                                                                     
*/

// Find offsets with bevel gears on the right side of the drive train
#define FL_WHEEL_OFFSET 0.474
#define FR_WHEEL_OFFSET 0.426 
#define BR_WHEEL_OFFSET 0.865 
#define BL_WHEEL_OFFSET 0.147 

// Assuming a rectangular drive train (input distance between center of wheels)
#define DRIVE_LENGTH 0.5906_m 
#define DRIVE_WIDTH 0.489_m 

// Encoder constants
#define DRIVE_MOTOR_GEAR_RATIO 7.36 
#define DRIVE_MOTOR_CIRCUMFERENCE 0.10469983 * M_PI
#define SPIN_MOTOR_GEAR_RATIO 15.43

// Swerve Module Wheel Spin PID Values
#define WHEEL_SPIN_KP 1
#define WHEEL_SPIN_KI 0
#define WHEEL_SPIN_KI_MAX 0 // In percent power
#define WHEEL_SPIN_KD 0
#define WHEEL_SPIN_TOLERANCE 3 // In Degrees
#define WHEEL_SPIN_VELOCITY_TOLERANCE 0.5 // In percent power
#define WHEEL_SPIN_MIN_SPEED 0 // In percent power
#define WHEEL_SPIN_MAX_SPEED 1.0 // In percent power

/*
     _         _                                                 ____                _              _       
    / \  _   _| |_ ___  _ __   ___  _ __ ___   ___  _   _ ___   / ___|___  _ __  ___| |_ __ _ _ __ | |_ ___ 
   / _ \| | | | __/ _ \| '_ \ / _ \| '_ ` _ \ / _ \| | | / __| | |   / _ \| '_ \/ __| __/ _` | '_ \| __/ __|
  / ___ \ |_| | || (_) | | | | (_) | | | | | | (_) | |_| \__ \ | |__| (_) | | | \__ \ || (_| | | | | |_\__ \
 /_/   \_\__,_|\__\___/|_| |_|\___/|_| |_| |_|\___/ \__,_|___/  \____\___/|_| |_|___/\__\__,_|_| |_|\__|___/

*/

/* Pure Odometry-Based Drive to Pose PID Values */
// Translational PID in the x and y direction
#define ODOMETRY_TRANSLATION_KP 1.1
#define ODOMETRY_TRANSLATION_KI 0
#define ODOMETRY_TRANSLATION_KI_MAX 0 // In percent power
#define ODOMETRY_TRANSLATION_KD 0
#define ODOMETRY_TRANSLATION_TOLERANCE 0.02 // In meters
#define ODOMETRY_TRANSLATION_VELOCITY_TOLERANCE 0.5 // In percent power
#define ODOMETRY_TRANSLATION_MIN_SPEED 0 // In percent power
#define ODOMETRY_TRANSLATION_MAX_SPEED 0.4 // In percent power
// Rotational PID to correct robot heading
