#include "Swerve.h"

class PhotonTagSwerve : public SwerveDrive
{
private:
    SwerveDriveOdometry<4> tagOdometry; /* An odometry class which returns the position of the robot using wheel encoder ticks*/
    photonlib::PhotonCamera camera; /* The link to a camera run by photonvision */
    frc::Transform3d robotToCam; /* The position of the camera in relation to the center of the robot */
    photonlib::RobotPoseEstimator poseEstimator; /* Photon Lib class to convert camera data to pose estimation */

public:
    PhotonTagSwerve();

    void ResetHeading();
    void ResetTagOdometry();
    void ResetTagOdometry(Pose2d position);
    Pose2d GetTagOdometryPose();
    void Update();
};