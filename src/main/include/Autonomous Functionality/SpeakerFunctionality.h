#include "Robot.h"
#include "Constants/FlywheelConstants.h"

#include "Intake.h"
#include "FlyWheel.h"
#include "Elevator.h"
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
    Intake *intake;
    Elevator *elevator;

    Timer shotTimer;
    double targetAnglerAngle;
    bool shootingNote;

    Translation2d diffDebug;

public:
    AutonomousShootingController(SwerveDriveAutonomousController *swerveDrive, FlywheelSystem *flyWheel_, Intake *intake_, Elevator *elevator_);

    bool TurnToSpeaker(AllianceColor allianceColor);
    bool TurnToSpeakerWhileDrivingMetersAndRadians(double xSpeed, double ySpeed, AllianceColor allianceColor);
    bool TurnToSpeakerWhileDriving(double xSpeed, double ySpeed, AllianceColor allianceColor);
    bool AngleFlywheelToSpeaker(AllianceColor allianceColor);
    bool SpinFlywheelForSpeaker(AllianceColor allianceColor);
    bool ClearElevatorForShot();
    bool ClearElevatorForShot(double anglerAngle);
    void BeginAimAndFire(AllianceColor allianceColor);
    bool AimAndFire(AllianceColor allianceColor);
    Translation2d GetDiffDebug();
};

#endif