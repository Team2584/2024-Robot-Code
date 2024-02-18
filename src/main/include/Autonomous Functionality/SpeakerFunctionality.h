#include "Robot.h"
#include "Constants/FlywheelConstants.h"

#include "FlyWheel.h"
#include "Autonomous Functionality/SwerveDriveAutoControl.h"
#include "Constants/FieldConstants.h"

#ifndef SPEAKER_FUNCTIONALITY_H
#define SPEAKER_FUNCTIONALITY_H


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
    void TurnToSpeakerWhileDriving(double xSpeed, double ySpeed);
    bool AngleFlywheelToSpeaker();
    bool AimAndFire();
};

#endif