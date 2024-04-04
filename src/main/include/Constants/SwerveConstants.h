#ifndef SWERVE_CONSTANTS_H // Ensures that this header file is only compiled once
#define SWERVE_CONSTANTS_H

/*
   ____                           _    ____                _              _       
  / ___| ___ _ __   ___ _ __ __ _| |  / ___|___  _ __  ___| |_ __ _ _ __ | |_ ___ 
 | |  _ / _ \ '_ \ / _ \ '__/ _` | | | |   / _ \| '_ \/ __| __/ _` | '_ \| __/ __|
 | |_| |  __/ | | |  __/ | | (_| | | | |__| (_) | | | \__ \ || (_| | | | | |_\__ \
  \____|\___|_| |_|\___|_|  \__,_|_|  \____\___/|_| |_|___/\__\__,_|_| |_|\__|___/
                                                                                                                     
*/

// Ports for different motors and sensors
#define FL_DRIVE_MOTOR_PORT 1
#define FL_SPIN__MOTOR_PORT 11
#define FR_DRIVE_MOTOR_PORT 2
#define FR_SPIN__MOTOR_PORT 12
#define BL_DRIVE_MOTOR_PORT 3
#define BL_SPIN__MOTOR_PORT 13
#define BR_DRIVE_MOTOR_PORT 4
#define BR_SPIN__MOTOR_PORT 14
#define FL_MAGNETIC_ENCODER_PORT 0
#define FR_MAGNETIC_ENCODER_PORT 1
#define BL_MAGNETIC_ENCODER_PORT 2
#define BR_MAGNETIC_ENCODER_PORT 3
#define PIGEON_IMU_PORT 6

// Find offsets with bevel gears on the right side of the drive train
#define FL_WHEEL_OFFSET 0.863  //0.2103 
#define FR_WHEEL_OFFSET 0.461  //0.5000 
#define BL_WHEEL_OFFSET 0.3605 //0.7116 
#define BR_WHEEL_OFFSET 0.386  //0.6294 

// Assuming a rectangular drive train (input distance between center of wheels)
#define DRIVE_LENGTH 0.5906_m 
#define DRIVE_WIDTH 0.489_m 

// Encoder constants
#define DRIVE_MOTOR_GEAR_RATIO 7.36 * 1.3908
#define DRIVE_MOTOR_CIRCUMFERENCE 0.11280562 * M_PI
#define SPIN_MOTOR_GEAR_RATIO 15.43

// Swerve Module Wheel Spin PID Values
#define WHEEL_SPIN_KP 0.008
#define WHEEL_SPIN_KI 0
#define WHEEL_SPIN_KI_MAX 0 // In percent power
#define WHEEL_SPIN_KD 0
#define WHEEL_SPIN_TOLERANCE 3 // In Degrees
#define WHEEL_SPIN_VELOCITY_TOLERANCE 0.5 // In percent power
#define WHEEL_SPIN_MIN_SPEED 0 // In percent power
#define WHEEL_SPIN_MAX_SPEED 1.0 // In percent power

//TalonFX Kraken Current Limits
#define SWERVE_TALON_STATOR_CURRENTLIMIT 60.0

/*
 __     ___     _                ____                _              _       
 \ \   / (_)___(_) ___  _ __    / ___|___  _ __  ___| |_ __ _ _ __ | |_ ___ 
  \ \ / /| / __| |/ _ \| '_ \  | |   / _ \| '_ \/ __| __/ _` | '_ \| __/ __|
   \ V / | \__ \ | (_) | | | | | |__| (_) | | | \__ \ || (_| | | | | |_\__ \
    \_/  |_|___/_|\___/|_| |_|  \____\___/|_| |_|___/\__\__,_|_| |_|\__|___/
                                                                            
*/

#define CAMERA_ONE_NAME "Top Camera"
#define CAMERA_ONE_X 11_in // Distance in meters from camera to center of robot forward/backward, forward positive
#define CAMERA_ONE_Y 11.625_in // Distance in meters from camera to center of robot left/right, left positive
#define CAMERA_ONE_Z 26.875_in // Camera's distance from ground in meters
#define CAMERA_ONE_X_ROTATION 0_deg // Camera's Rotation around robot's X axis (tilted up/down)
#define CAMERA_ONE_Y_ROTATION -47_deg // Camera's Rotation around robot's Y axis (rotated clockwise/counterclocwise)
#define CAMERA_ONE_Z_ROTATION 0_deg // Camera's Rotation around robot's Z axis (tilted left/right)

#define CAMERA_TWO_NAME "Bottom Camera"
#define CAMERA_TWO_X 12.5_in // Distance in meters from camera to center of robot forward/backward, forward positive
#define CAMERA_TWO_Y 11.625_in // Distance in meters from camera to center of robot left/right, left positive
#define CAMERA_TWO_Z 25.25_in // Camera's distance from ground in meters
#define CAMERA_TWO_X_ROTATION 0_deg // Camera's Rotation around robot's X axis (tilted up/down)
#define CAMERA_TWO_Y_ROTATION -17_deg // Camera's Rotation around robot's Y axis (rotated clockwise/counterclocwise)
#define CAMERA_TWO_Z_ROTATION 0_deg // Camera's Rotation around robot's Z axis (tilted left/right)

#define CAMERA_THREE_NAME "Left Camera"
#define CAMERA_THREE_X 10.25_in // Distance in meters from camera to center of robot forward/backward, forward positive
#define CAMERA_THREE_Y 12.25_in // Distance in meters from camera to center of robot left/right, left positive
#define CAMERA_THREE_Z 16.75_in // Camera's distance from ground in meters
#define CAMERA_THREE_X_ROTATION 0_deg // Camera's Rotation around robot's X axis (tilted up/down)
#define CAMERA_THREE_Y_ROTATION 0_deg // Camera's Rotation around robot's Y axis (rotated clockwise/counterclocwise)
#define CAMERA_THREE_Z_ROTATION -90_deg // Camera's Rotation around robot's Z axis (tilted left/right)

#define APRILTAG_CONFIDENCE_X 0.1
#define APRILTAG_CONFIDENCE_Y 0.1
#define APRILTAG_CONFIDENCE_ROTATION 5.0

#define LEFT_CAM_CONFIDENCE_X 0.15
#define LEFT_CAM_CONFIDENCE_Y 0.15
#define LEFT_CAM_CONFIDENCE_ROTATION 10.0

/*
     _         _                                                 ____                _              _       
    / \  _   _| |_ ___  _ __   ___  _ __ ___   ___  _   _ ___   / ___|___  _ __  ___| |_ __ _ _ __ | |_ ___ 
   / _ \| | | | __/ _ \| '_ \ / _ \| '_ ` _ \ / _ \| | | / __| | |   / _ \| '_ \/ __| __/ _` | '_ \| __/ __|
  / ___ \ |_| | || (_) | | | | (_) | | | | | | (_) | |_| \__ \ | |__| (_) | | | \__ \ || (_| | | | | |_\__ \
 /_/   \_\__,_|\__\___/|_| |_|\___/|_| |_| |_|\___/ \__,_|___/  \____\___/|_| |_|___/\__\__,_|_| |_|\__|___/

*/

enum PoseEstimationType
{
    PureOdometry = 0,
    TagBased = 1,
    NoteBased = 2
};


/* Drive to Pose PID Values */
// Translational PID in the x and y direction
#define DTP_TRANSLATION_KP 0.33
#define DTP_TRANSLATION_KI 0
#define DTP_TRANSLATION_KI_MAX 0 // In percent power
#define DTP_TRANSLATION_KD 0
#define DTP_TRANSLATION_TOLERANCE 0.04 // In meters
#define DTP_TRANSLATION_VELOCITY_TOLERANCE 0.5 // In percent power
#define DTP_TRANSLATION_MIN_SPEED 0.02 // In percent power
#define DTP_TRANSLATION_MAX_SPEED 0.6 // In percent power
// Rotational PID to correct robot heading
#define DTP_ROTATION_KP 0.385
#define DTP_ROTATION_KI 0
#define DTP_ROTATION_KI_MAX 0 // In percent power
#define DTP_ROTATION_KD 0
#define DTP_ROTATION_TOLERANCE 0.05 // In radians
#define DTP_ROTATION_VELOCITY_TOLERANCE 0.5 // In percent power
#define DTP_ROTATION_MIN_SPEED 0.025 // In percent powerw
#define DTP_ROTATION_MAX_SPEED 0.6 // In percent power

/* Trajectory following PID Values */
// Translational PID in the x and y direction
#define TRAJECTORY_TRANSLATION_KP 3.5
#define TRAJECTORY_TRANSLATION_KI 0
#define TRAJECTORY_TRANSLATION_KI_MAX 0 // In percent power
#define TRAJECTORY_TRANSLATION_KD 0
#define TRAJECTORY_TRANSLATION_TOLERANCE 0.02 // In meters
#define TRAJECTORY_TRANSLATION_VELOCITY_TOLERANCE 0.5 // In meters per second
#define TRAJECTORY_TRANSLATION_MIN_SPEED 0 // In meters per second
#define TRAJECTORY_TRANSLATION_MAX_SPEED 2.5 // In meters per second
// Rotational PID to correct robot heading 
#define TRAJECTORY_ROTATION_KP 2
#define TRAJECTORY_ROTATION_KI 0
#define TRAJECTORY_ROTATION_KI_MAX 0 // In percent power
#define TRAJECTORY_ROTATION_KD 0
#define TRAJECTORY_ROTATION_TOLERANCE 0.07 // In radians
#define TRAJECTORY_ROTATION_VELOCITY_TOLERANCE 0.5 // In radians per second
#define TRAJECTORY_ROTATION_MIN_SPEED 0 // In radians per second
#define TRAJECTORY_ROTATION_MAX_SPEED 3 // In radians per second

/* Automatically Picking Up Note PID Values */
// Translational PID in the x direction
#define NOTE_X_KP 0.1
#define NOTE_X_KI 0
#define NOTE_X_KI_MAX 0 // In percent power
#define NOTE_X_KD 0
#define NOTE_X_TOLERANCE 0.02 // In meters
#define NOTE_X_VELOCITY_TOLERANCE 0.5 // In percent power
#define NOTE_X_MIN_SPEED 0 // In percent power
#define NOTE_X_MAX_SPEED 0.4 // In percent power
// Translational PID in the y direction
#define NOTE_Y_KP 0.5
#define NOTE_Y_KI 0
#define NOTE_Y_KI_MAX 0 // In percent power
#define NOTE_Y_KD 0
#define NOTE_Y_TOLERANCE 0.02 // In meters
#define NOTE_Y_VELOCITY_TOLERANCE 0.5 // In percent power
#define NOTE_Y_MIN_SPEED 0 // In percent power
#define NOTE_Y_MAX_SPEED 0.4 // In percent power
// Rotational PID to correct robot heading
#define NOTE_ROTATION_KP 0.28
#define NOTE_ROTATION_KI 0
#define NOTE_ROTATION_KI_MAX 0 // In percent power
#define NOTE_ROTATION_KD 0
#define NOTE_ROTATION_TOLERANCE 0.15 // In radians
#define NOTE_ROTATION_VELOCITY_TOLERANCE 100 // In percent power
#define NOTE_ROTATION_MIN_SPEED 0.02 // In percent power
#define NOTE_ROTATION_MAX_SPEED 0.6 // In percent power

#endif // SWERVE_CONSTANTS_H