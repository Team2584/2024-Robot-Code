#ifndef PHOTON_SWERVE_H // Ensures that this header file is only compiled once
#define PHOTON_SWERVE_H

#include "Swerve.h"

/**
 * This class inherits from the base SwerveDrive class, but adds the functionality of calculating odometry using the photonvision library, a camera, and april tags
 */
class PhotonTagSwerve : public SwerveDrive
{
private:
    SwerveDrivePoseEstimator<4> tagOdometry; /* An odometry class which returns the position of the robot using wheel encoder ticks*/
    Transform3d robotToCam; /* The Position and rotation of the camera on the robot */
    photonlib::PhotonCamera camera; /* The Camera reading the data */
    photonlib::PhotonPoseEstimator poseEstimator; /* Photon Lib class to convert camera data to pose estimation */
    Pose3d prevEstimatedPose; /* The previous pose of the robot */

public:
    PhotonTagSwerve();

    void ResetHeading();
    void ResetTagOdometry();
    void ResetTagOdometry(Pose2d position);
    void AddVisionMeasurement(Pose2d measurement, units::second_t timeStamp);
    bool TagInView();
    Transform3d GetTagReading();
    Pose2d GetTagOdometryPose();
    void UpdateTagOdometry();
    void Update();
};

#endif // PHOTON_SWERVE_H