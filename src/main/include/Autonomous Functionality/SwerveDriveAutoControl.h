#include "Robot.h"
#include "PhotonTagSwerve.h"
#include "Constants/SwerveConstants.h"

/**
 * This class inherits from the base SwerveDrive class, but adds the functionality of calculating odometry using the photonvision library, a camera, and april tags
 */
class SwerveDriveAutonomousController
{
private:
    SwerveDrive *baseSwerveDrive = NULL; /* A reference to our base swerve drive template. */
    PhotonTagSwerve *photonTagSwerve = NULL; /* A reference to our photonlib swerve drive template. */
    PID xPIDController, yPIDController, rotationPIDController; /* PID Controllers to move in various directions */

public:
    SwerveDriveAutonomousController(SwerveDrive *swerveDrive);
    SwerveDriveAutonomousController(PhotonTagSwerve *swerveDrive);

    bool DriveToPose(OdometryType odometryType, Pose2d target);
};