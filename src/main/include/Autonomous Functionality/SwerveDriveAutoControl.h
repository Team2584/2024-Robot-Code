#include "Robot.h"
#include "Constants/SwerveConstants.h"

/**
 * This class inherits from the base SwerveDrive class, but adds the functionality of calculating odometry using the photonvision library, a camera, and april tags
 */
class SwerveDriveAutonomousController
{
private:
    SwerveDrive *swerveDrive;
    PID xPIDController, yPIDController, rotationPIDController; /* PID Controllers to move in various directions */

public:
    SwerveDriveAutonomousController(SwerveDrive *swerveDrive_);

    bool DriveToPose(OdometryType odometryType, Pose2d target);
};