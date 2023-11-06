#include "Swerve.h"

/**
 * This class inherits from the base SwerveDrive class, but adds the functionality of calculating odometry using the photonvision library, a camera, and april tags
 */
class PhotonTagSwerve : public SwerveDrive
{
private:
    SwerveDrivePoseEstimator<4> tagOdometry; /* An odometry class which returns the position of the robot using wheel encoder ticks*/
    Transform3d robotToCam; /* The Position and rotation of the camera on the robot */
    photonlib::PhotonPoseEstimator poseEstimator; /* Photon Lib class to convert camera data to pose estimation */
    Pose3d prevEstimatedPose; /* The previous pose of the robot */

public:
    PhotonTagSwerve();

    void ResetHeading();
    void ResetTagOdometry();
    void ResetTagOdometry(Pose2d position);
    Pose2d GetTagOdometryPose();
    void UpdateTagOdometry();
    void Update();
};