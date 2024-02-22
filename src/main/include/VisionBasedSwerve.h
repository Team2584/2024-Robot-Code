#ifndef VISION_SWERVE_H // Ensures that this header file is only compiled once
#define VISION_SWERVE_H

#include "Swerve.h"
#include "LimeLight.h"
#include "Constants/FieldConstants.h"

/**
 * This class inherits from the base SwerveDrive class, but adds the functionality of calculating odometry using the photonvision library, a camera, and april tags
 */
class VisionSwerve : public SwerveDrive
{
private:
    SwerveDrivePoseEstimator<4> tagOdometry; /* An odometry class which returns the position of the robot using wheel encoder ticks*/
    Transform3d robotToCam; /* The Position and rotation of the camera on the robot */
    photon::PhotonCamera camera; /* The Camera reading the data */
    photon::PhotonPoseEstimator poseEstimator; /* Photon Lib class to convert camera data to pose estimation */
    Pose3d prevEstimatedPose; /* The previous pose of the robot */

    SwerveDriveOdometry<4> noteOdometry; /* An odometry class which returns the position of the robot using wheel encoder ticks */ 
    Limelight limelight;
    

public:
    VisionSwerve();
    void ResetHeading();

    void ResetTagOdometry();
    void ResetTagOdometry(Pose2d position);
    void AddVisionMeasurement(Pose2d measurement, units::second_t timeStamp);
    bool TagInView();
    Transform3d GetTagReading();
    Pose2d GetTagOdometryPose();
    void UpdateTagOdometry();
    void DriveSwervePercentTagOriented(double FwdDriveSpeed, double StrafeDriveSpeed, double TurnSpeed);

    void UpdateLimelightConnection();
    void PrintLimelightSanityCheck();
    bool NoteInView();
    void ResetNoteOdometry();
    void ResetNoteOdometry(Pose2d position);
    Translation2d GetNotePosition();
    Pose2d GetNoteOdometryPose();
    void UpdateNoteOdometry();

    void Update();
};

#endif // Vision_SWERVE_H