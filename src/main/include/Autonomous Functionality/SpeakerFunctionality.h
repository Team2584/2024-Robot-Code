#include "Robot.h"
#include "Constants/FlywheelConstants.h"

#include "FlyWheel.h"
#include "Autonomous Functionality/SwerveDriveAutoControl.h"
#include "Constants/FieldConstants.h"

/**
 * This class inherits from the base SwerveDrive class, but adds the functionality of calculating odometry using the photonvision library, a camera, and april tags
 */
class AutonomousShootingController
{
private:
    SwerveDriveAutonomousController *swerveDrive;
    FlywheelSystem *flyWheel;

public:
    AutonomousShootingController(SwerveDriveAutonomousController *swerveDrive, FlywheelSystem *flyWheel_);

    bool TurnToSpeaker();
    bool AngleFlywheelToSpeaker();
    bool AimAndFire();
};