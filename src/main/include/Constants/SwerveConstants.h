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
#define FL_MAGNETIC_ENCODER_PORT 8
#define FR_MAGNETIC_ENCODER_PORT 6
#define BL_MAGNETIC_ENCODER_PORT 9
#define BR_MAGNETIC_ENCODER_PORT 7
#define PIGEON_IMU_PORT 6

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
#define DRIVE_MOTOR_CIRCUMFERENCE 0.11280562 * M_PI
#define SPIN_MOTOR_GEAR_RATIO 15.43

// Swerve Module Wheel Spin PID Values
#define WHEEL_SPIN_KP 0.013
#define WHEEL_SPIN_KI 0
#define WHEEL_SPIN_KI_MAX 0 // In percent power
#define WHEEL_SPIN_KD 0
#define WHEEL_SPIN_TOLERANCE 3 // In Degrees
#define WHEEL_SPIN_VELOCITY_TOLERANCE 0.5 // In percent power
#define WHEEL_SPIN_MIN_SPEED 0 // In percent power
#define WHEEL_SPIN_MAX_SPEED 1.0 // In percent power

/*
 __     ___     _                ____                _              _       
 \ \   / (_)___(_) ___  _ __    / ___|___  _ __  ___| |_ __ _ _ __ | |_ ___ 
  \ \ / /| / __| |/ _ \| '_ \  | |   / _ \| '_ \/ __| __/ _` | '_ \| __/ __|
   \ V / | \__ \ | (_) | | | | | |__| (_) | | | \__ \ || (_| | | | | |_\__ \
    \_/  |_|___/_|\___/|_| |_|  \____\___/|_| |_|___/\__\__,_|_| |_|\__|___/
                                                                            
*/

// List of april tag's IDs and their position on the field in comparison to origin
const std::vector<frc::AprilTag> fieldTagLayout = {
    {0, Pose3d(1_m, 0_m, 1.15_m, Rotation3d(0_deg, 0_deg, 180_deg))},
    {1, Pose3d(5_m, 5_m, 5_m, Rotation3d())}};
const AprilTagFieldLayout aprilTags = AprilTagFieldLayout(fieldTagLayout, 54_ft, 27_ft); // frc field is 54 ft by 27 ft

#define CAMERA_ONE_NAME "photonvision"
#define CAMERA_ONE_X 16_in // Distance in meters from camera to center of robot forward/backward, forward positive
#define CAMERA_ONE_Y -0.6_in // Distance in meters from camera to center of robot left/right, left positive
#define CAMERA_ONE_Z 37_in // Camera's distance from ground in meters
#define CAMERA_ONE_X_ROTATION 0_deg // Camera's Rotation around robot's X axis (tilted up/down)
#define CAMERA_ONE_Y_ROTATION 0_deg // Camera's Rotation around robot's Y axis (rotated clockwise/counterclocwise)
#define CAMERA_ONE_Z_ROTATION 0_deg // Camera's Rotation around robot's Z axis (tilted left/right)

#define APRILTAG_CONFIDENCE_X 0.5
#define APRILTAG_CONFIDENCE_Y 0.5
#define APRILTAG_CONFIDENCE_ROTATION 10

/*
     _         _                                                 ____                _              _       
    / \  _   _| |_ ___  _ __   ___  _ __ ___   ___  _   _ ___   / ___|___  _ __  ___| |_ __ _ _ __ | |_ ___ 
   / _ \| | | | __/ _ \| '_ \ / _ \| '_ ` _ \ / _ \| | | / __| | |   / _ \| '_ \/ __| __/ _` | '_ \| __/ __|
  / ___ \ |_| | || (_) | | | | (_) | | | | | | (_) | |_| \__ \ | |__| (_) | | | \__ \ || (_| | | | | |_\__ \
 /_/   \_\__,_|\__\___/|_| |_|\___/|_| |_| |_|\___/ \__,_|___/  \____\___/|_| |_|___/\__\__,_|_| |_|\__|___/

*/

enum OdometryType
{
    PureOdometry = 0,
    TagBased = 1
};


/* Pure Odometry-Based Drive to Pose PID Values */
// Translational PID in the x and y direction
#define ODOMETRY_TRANSLATION_KP 0.5
#define ODOMETRY_TRANSLATION_KI 0
#define ODOMETRY_TRANSLATION_KI_MAX 0 // In percent power
#define ODOMETRY_TRANSLATION_KD 0
#define ODOMETRY_TRANSLATION_TOLERANCE 0.02 // In meters
#define ODOMETRY_TRANSLATION_VELOCITY_TOLERANCE 0.5 // In percent power
#define ODOMETRY_TRANSLATION_MIN_SPEED 0 // In percent power
#define ODOMETRY_TRANSLATION_MAX_SPEED 0.2 // In percent power
// Rotational PID to correct robot heading
#define ODOMETRY_ROTATION_KP 0.8
#define ODOMETRY_ROTATION_KI 0
#define ODOMETRY_ROTATION_KI_MAX 0 // In percent power
#define ODOMETRY_ROTATION_KD 0
#define ODOMETRY_ROTATION_TOLERANCE 0.07 // In radians
#define ODOMETRY_ROTATION_VELOCITY_TOLERANCE 0.5 // In percent power
#define ODOMETRY_ROTATION_MIN_SPEED 0 // In percent power
#define ODOMETRY_ROTATION_MAX_SPEED 0.2 // In percent power

#endif // SWERVE_CONSTANTS_H