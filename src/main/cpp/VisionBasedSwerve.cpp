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
      noteOdometry{kinematics, Rotation2d(units::degree_t{GetIMUHeading()}), GetSwerveModulePositions(), Pose2d()},
      networkTableInstance{nt::NetworkTableInstance::GetDefault()},
      visionTable{networkTableInstance.GetTable("vision")},
      sanityTopic{visionTable->GetDoubleTopic("sanitycheck")},
      sanityEntry{sanityTopic.GetEntry(-1)},
      connectedTopic{visionTable->GetDoubleTopic("connected")},
      connectedEntry{connectedTopic.GetEntry(false)},
      notePosTopic{visionTable->GetDoubleTopic("ringPos")},
      notePoseSubscriber{notePosTopic.Subscribe({})}
{
    tagOdometry.SetVisionMeasurementStdDevs(wpi::array(APRILTAG_CONFIDENCE_X, APRILTAG_CONFIDENCE_Y, APRILTAG_CONFIDENCE_ROTATION));

    networkTableInstance.StartServer();
    connectedEntry.Set(true);
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
    return result.HasTargets();
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
}

void VisionSwerve::PrintRaspiSanityCheck()
{
    SmartDashboard::PutNumber("Raspi Sanity Check", sanityEntry.Get());
}

bool VisionSwerve::NoteInView()
{
    return noteInViewSubscriber.Get();
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
    auto array = notePoseSubscriber.Get();
    units::meter_t xPos = units::meter_t{array[0]};
    units::meter_t yPos = units::meter_t{array[1]};
    return Translation2d(xPos, yPos);
}

Pose2d VisionSwerve::GetNoteOdometryPose()
{
    return noteOdometry.GetPose();
}

void VisionSwerve::UpdateNoteOdometry()
{
    if (NoteInView())
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