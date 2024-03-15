#include "VisionBasedSwerve.h"

/**
 * Instantiates a Photolib Tag-based Swerve Drive
 */
VisionSwerve::VisionSwerve()
    : SwerveDrive(),
      tagOdometry{kinematics, Rotation2d(units::degree_t{GetIMUHeading()}), GetSwerveModulePositions(), Pose2d()},
      robotToCam{frc::Translation3d{CAMERA_ONE_X, CAMERA_ONE_Y, CAMERA_ONE_Z}, frc::Rotation3d{CAMERA_ONE_X_ROTATION, CAMERA_ONE_Y_ROTATION, CAMERA_ONE_Z_ROTATION}},
      camera{CAMERA_ONE_NAME},
      poseEstimator{APRIL_TAGS, photon::LOWEST_AMBIGUITY, photon::PhotonCamera(CAMERA_ONE_NAME), robotToCam},
      robotToCam2{frc::Translation3d{CAMERA_TWO_X, CAMERA_TWO_Y, CAMERA_TWO_Z}, frc::Rotation3d{CAMERA_TWO_X_ROTATION, CAMERA_TWO_Y_ROTATION, CAMERA_TWO_Z_ROTATION}},
      camera2{CAMERA_TWO_NAME},
      poseEstimator2{APRIL_TAGS, photon::LOWEST_AMBIGUITY, photon::PhotonCamera(CAMERA_TWO_NAME), robotToCam2},
      noteOdometry{kinematics, Rotation2d(units::degree_t{GetIMUHeading()}), GetSwerveModulePositions(), Pose2d()},
      networkTableInstance{nt::NetworkTableInstance::GetDefault()},
      visionTable{networkTableInstance.GetTable("limelight")},
      limelight{visionTable}
{
    tagOdometry.SetVisionMeasurementStdDevs(wpi::array(APRILTAG_CONFIDENCE_X, APRILTAG_CONFIDENCE_Y, APRILTAG_CONFIDENCE_ROTATION));

    networkTableInstance.StartServer();
}

/**
 * Resets Robot heading to facing away from the driver
 */
void VisionSwerve::ResetHeading()
{
    SwerveDrive::ResetHeading();

    Pose2d currentPose = GetTagOdometryPose();
    ResetTagOdometry(Pose2d(currentPose.Translation(), Rotation2d(0_deg)));

    currentPose = GetNoteOdometryPose();
    ResetNoteOdometry(Pose2d(currentPose.Translation(), Rotation2d(0_deg)));
}

/**
 * Resets Tag-based Odometry to (0,0) facing away from the driver
 */
void VisionSwerve::ResetTagOdometry()
{
    ResetTagOdometry(Pose2d(0_m, 0_m, Rotation2d(0_deg)));
}

/**
 * Resets Tag-based Odometry to a given position
 */
void VisionSwerve::ResetTagOdometry(Pose2d position)
{
    prevEstimatedPose = Pose3d(position);
    tagOdometry.ResetPosition(Rotation2d(units::degree_t{GetIMUHeading()}),
                              GetSwerveModulePositions(), position);
}

void VisionSwerve::AddVisionMeasurement(Pose2d measurement, units::second_t timeStamp)
{
    tagOdometry.AddVisionMeasurement(measurement, timeStamp);
}

/*
 * Returns true if there is a tag in the current view of the camera
 */
bool VisionSwerve::TagInView()
{
    photon::PhotonPipelineResult result = camera.GetLatestResult();
    photon::PhotonPipelineResult result2 = camera2.GetLatestResult();
    return result.HasTargets() || result2.HasTargets();
}

/**
 * Returns the position of the most recent tag seen in relation to the robot
 */
Transform3d VisionSwerve::GetTagReading()
{
    photon::PhotonPipelineResult result = camera.GetLatestResult();
    if (result.HasTargets())
        return result.GetBestTarget().GetBestCameraToTarget();
    else
        return Transform3d();
}


/**
 * Finds the Pose of the robot using april tag data and odometry
 */
Pose2d VisionSwerve::GetTagOdometryPose()
{
    return tagOdometry.GetEstimatedPosition();
}

/*
 * Updates the Position estimation of the tag-based odometry using visino data and encoder counts
 */
void VisionSwerve::UpdateTagOdometry()
{
    // Update encoder counts of odometry
    tagOdometry.Update(units::degree_t{GetIMUHeading()}, GetSwerveModulePositions());

    // Add april tag data
    poseEstimator.SetReferencePose(prevEstimatedPose);
    optional<photon::EstimatedRobotPose> possibleResult = poseEstimator.Update();
    if (possibleResult.has_value()) 
    {
        photon::EstimatedRobotPose result = possibleResult.value();
        AddVisionMeasurement(result.estimatedPose.ToPose2d(), result.timestamp);
        prevEstimatedPose = result.estimatedPose;
    } 

    // Add april tag data
    poseEstimator2.SetReferencePose(prevEstimatedPose);
    possibleResult = poseEstimator2.Update();
    if (possibleResult.has_value()) 
    {
        photon::EstimatedRobotPose result = possibleResult.value();
        AddVisionMeasurement(result.estimatedPose.ToPose2d(), result.timestamp);
        prevEstimatedPose = result.estimatedPose;
    } 

    if (limelight.GetLimelightPipeline() == LimelightPipeline::)
}

/**
 * Drives the swerve drive, field oriented (in relation to the driver's pov) with an x y and spin.
 * @param FwdDriveSpeed The speed the robot should move forward and back, positive being forward, in percentage (0 - 1.0)
 * @param StrafeDriveSpeed The speed the robot should move left and right, positive being right, in percentage (0 - 1.0)
 * @param TurnSpeed The speed the robot should turn left and right, positive being counterclockwise, in percentage (0 - 1.0)
 */
void VisionSwerve::DriveSwervePercentTagOriented(double FwdDriveSpeed, double StrafeDriveSpeed, double TurnSpeed)
{
    // Converts our field oriented speeds to robot oriented, by using trig (rotation matrix) with the current robot angle.
    double angle = -1 * GetTagOdometryPose().Rotation().Radians().value(); // Angle * -1 because rotation matrices rotate clockwise
    double oldFwd = FwdDriveSpeed;
    FwdDriveSpeed = FwdDriveSpeed * cos(angle) - StrafeDriveSpeed * sin(angle);
    StrafeDriveSpeed = oldFwd * sin(angle) + StrafeDriveSpeed * cos(angle);

    DriveSwervePercentNonFieldOriented(FwdDriveSpeed, StrafeDriveSpeed, TurnSpeed);
}

void VisionSwerve::DriveSwerveTagOrientedMetersAndRadians(double FwdDriveSpeed, double StrafeDriveSpeed, double TurnSpeed)
{
    DriveSwervePercentTagOriented(VelocityToPercent(FwdDriveSpeed), VelocityToPercent(StrafeDriveSpeed), AngularVelocityToPercent(TurnSpeed));
}


void VisionSwerve::UpdateLimelightConnection()
{
   
}

bool VisionSwerve::NoteInView()
{
    return limelight.noteInViewSubscriber.Get();
}

void VisionSwerve::ResetNoteOdometry()
{
    ResetNoteOdometry(Pose2d(0_m, 0_m, Rotation2d(0_rad)));
}
void VisionSwerve::ResetNoteOdometry(Pose2d position)
{
    noteOdometry.ResetPosition(Rotation2d(units::degree_t{GetIMUHeading()}),
                               GetSwerveModulePositions(), position);
}

Translation2d VisionSwerve::GetNotePosition()
{
    return limelight.GetNotePose();
}

Pose2d VisionSwerve::GetNoteOdometryPose()
{
    return noteOdometry.GetPose();
}

double VisionSwerve::GetNoteTx()
{
    return limelight.GetNoteTx();
}

double VisionSwerve::GetNoteTy()
{
    return limelight.GetNoteTy();
}

void VisionSwerve::UpdateNoteOdometry()
{
    // Update encoder counts of odometry
    noteOdometry.Update(units::degree_t{GetIMUHeading()}, GetSwerveModulePositions());

    // Incorporate Note Data If Note is more that 0.7_m from robot
    if (NoteInView() && GetNotePosition().X() < -0.7_m)
    {
        ResetNoteOdometry(Pose2d(GetNotePosition(), GetOdometryPose().Rotation()));
    }
}

/*
 * Update all methods of odometry
 */
void VisionSwerve::Update()
{
    SwerveDrive::Update();
    UpdateTagOdometry();
    UpdateNoteOdometry();
}