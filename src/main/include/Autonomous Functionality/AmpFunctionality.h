#include "Robot.h"
#include "Constants/FieldConstants.h"
#include "Constants/ElevatorConstants.h"

#include "NoteController.h"
#include "Autonomous Functionality/SwerveDriveAutoControl.h"

#ifndef AMP_FUNCTIONALITY_H
#define AMP_FUNCTIONALITY_H

/**
 * This class inherits from the base SwerveDrive class, but adds the functionality of calculating odometry using the photonvision library, a camera, and april tags
 */
class AutonomousAmpingController
{
private:
    SwerveDriveAutonomousController *swerveDrive;
    NoteController *noteController;

    bool centeredOnAmp = false;
    bool drivingIntoAmp = false;
    Timer ampTimer;

public:
    AutonomousAmpingController(SwerveDriveAutonomousController *swerveDrive_, NoteController *noteController_);

    void BeginDriveToAmp(AllianceColor allianceColor);
    bool DriveToAmp(AllianceColor allianceColor);
    void BeginScoreInAmp(AllianceColor allianceColor);
    bool ScoreInAmp(AllianceColor allianceColor);
};

#endif