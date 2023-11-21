#include "PhotonTagSwerve.h"

/**
 * Instantiates a Photolib Tag-based Swerve Drive
 */
PhotonTagSwerve::PhotonTagSwerve()
    : SwerveDrive(),
      tagOdometry{kinematics, Rotation2d(units::degree_t{GetIMUHeading()}), GetSwerveModulePositions(), Pose2d()},
      robotToCam{frc::Translation3d{CAMERA_ONE_X, CAMERA_ONE_Y, CAMERA_ONE_Z}, frc::Rotation3d{CAMERA_ONE_X_ROTATION, CAMERA_ONE_Y_ROTATION, CAMERA_ONE_Z_ROTATION}},
      camera{CAMERA_ONE_NAME},
      poseEstimator{aprilTags, photonlib::CLOSEST_TO_REFERENCE_POSE, photonlib::PhotonCamera(CAMERA_ONE_NAME), robotToCam}
{
    tagOdometry.SetVisionMeasurementStdDevs(wpi::array(APRILTAG_CONFIDENCE_X, APRILTAG_CONFIDENCE_Y, APRILTAG_CONFIDENCE_ROTATION));
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
    prevEstimatedPose = Pose3d(position);
    tagOdometry.ResetPosition(Rotation2d(units::degree_t{GetIMUHeading()}),
                              GetSwerveModulePositions(),
                              frc::Pose2d(Pose2d(position.Y(), position.X(), position.Rotation())));
}

void PhotonTagSwerve::AddVisionMeasurement(Pose2d measurement, units::second_t timeStamp)
{
    //Remove Rotation from vision measurement because IMU is very accurate
    measurement = Pose2d(measurement.X(), measurement.Y(), Rotation2d(units::degree_t{GetIMUHeading()}));
    tagOdometry.AddVisionMeasurement(measurement, timeStamp);
}

/*
 * Returns true if there is a tag in the current view of the camera
 */
bool PhotonTagSwerve::TagInView()
{
    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    return result.HasTargets();
}

/**
 * Returns the position of the most recent tag seen in relation to the robot
 */
Transform3d PhotonTagSwerve::GetTagReading()
{
    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    if (result.HasTargets())
        return result.GetBestTarget().GetBestCameraToTarget();
    else
        return Transform3d();
}


/**
 * Finds the Pose of the robot using april tag data and odometry
 */
Pose2d PhotonTagSwerve::GetTagOdometryPose()
{
    Pose2d pose = tagOdometry.GetEstimatedPosition();
    return Pose2d(pose.Y(), pose.X(), pose.Rotation());
}

/*
 * Updates the Position estimation of the tag-based odometry using visino data and encoder counts
 */
void PhotonTagSwerve::UpdateTagOdometry()
{
    // Update encoder counts of odometry
    tagOdometry.Update(units::degree_t{GetIMUHeading()}, GetSwerveModulePositions());

    // Add april tag data
    poseEstimator.SetReferencePose(prevEstimatedPose);
    units::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
    optional<photonlib::EstimatedRobotPose> possibleResult = poseEstimator.Update();
    if (possibleResult.has_value()) 
    {
        photonlib::EstimatedRobotPose result = possibleResult.value();
        AddVisionMeasurement(result.estimatedPose.ToPose2d(), result.timestamp);
        prevEstimatedPose = result.estimatedPose;
    } 
}

/*
 * Update all methods of odometry
 */
void PhotonTagSwerve::Update()
{
    SwerveDrive::Update();
    UpdateTagOdometry();
}