#include "PhotonTagSwerve.h"

/**
 * Instantiates a Photolib Tag-based Swerve Drive
 */
PhotonTagSwerve::PhotonTagSwerve()
    : SwerveDrive(),
      tagOdometry{kinematics, Rotation2d(0_deg), GetSwerveModulePositions()},
      camera{CAMERA_ONE_NAME},
      robotToCam{frc::Translation3d{CAMERA_ONE_X, CAMERA_ONE_Y, CAMERA_ONE_Z}, frc::Rotation3d{CAMERA_ONE_X_ROTATION, CAMERA_ONE_Y_ROTATION, CAMERA_ONE_Z_ROTATION}},
      poseEstimator{}
{
}

/**
 * Resets Robot heading to facing away from the driver
 */
void PhotonTagSwerve::ResetHeading()
{
    SwerveDrive::ResetHeading();

    Pose2d currentPose = GetTagOdometryPose();
    ResetTagOdometry(Pose2d(currentPose.X(), currentPose.Y(), Rotation2d(0_deg)));
}

/**
 * Resets Tag-based Odometry to (0,0) facing away from the driver
 */
void PhotonTagSwerve::ResetTagOdometry()
{
    ResetTagOdometry(Pose2d(0_m, 0_m, Rotation2d(0_deg)));
}

/**
 * Resets Tag-based Odometry to a given position
 */
void PhotonTagSwerve::ResetTagOdometry(Pose2d position)
{
    tagOdometry.ResetPosition(Rotation2d(units::degree_t{GetIMUHeading()}),
                              GetSwerveModulePositions(),
                              frc::Pose2d(Pose2d(position.Y(), position.X(), position.Rotation())));
}

/**
 * Finds the Pose of the robot using april tag data and odometry
 */
Pose2d PhotonTagSwerve::GetTagOdometryPose()
{
    Pose2d pose = tagOdometry.GetPose();
    return Pose2d(pose.Y(), pose.X(), pose.Rotation());
}

void PhotonTagSwerve::Update()
{
    SwerveDrive::Update();
}