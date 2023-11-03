#include "Swerve.h"

class PhotonTagSwerve : public SwerveDrive
{
private:
    SwerveDriveOdometry<4> *odometry; /* An odometry class which returns the position of the robot using wheel encoder ticks*/
    photonlib::PhotonCamera camera; /* The link to a camera run by photonvision */

public:
    PhotonTagSwerve();

    void ResetTagOdometry();
    void ResetTagOdometry(Pose2d position);
    void ResetAllOdometry();
    void ResetAllOdometry(Pose2d position);
    Pose2d GetTagOdometryPose();
    void Update();
};