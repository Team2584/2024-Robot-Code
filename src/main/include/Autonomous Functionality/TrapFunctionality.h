#include "Robot.h"
#include "Constants/FieldConstants.h"
#include "Constants/ElevatorConstants.h"
#include "Constants/ClimbConstants.h"

#include "NoteController.h"
#include "Climb.h"
#include "Autonomous Functionality/SwerveDriveAutoControl.h"

#ifndef TRAP_FUNCTIONALITY_H
#define TRAP_FUNCTIONALITY_H

/**
 * This class inherits from the base SwerveDrive class, but adds the functionality of calculating odometry using the photonvision library, a camera, and april tags
 */
class AutonomousTrapController
{
private:
    
    SwerveDriveAutonomousController *swerveController;
    NoteController *noteController;
    Elevator *elevator;
    Climb *climb;
    Intake *intake;
    Timer climbTimer;

public:

    AutonomousTrapController(SwerveDriveAutonomousController *swerveController_, NoteController *noteController_, Elevator *elevator_, Climb *climb_, Intake *intake_);

    bool PrepareClimb();
    bool AttachHooks();
    bool ClimbToTrap();
    bool ScoreInTrap();
    bool LockRotationToNearestClimbPose(AllianceColor allianceColor, double xSpeed, double ySpeed);
    bool DriveToNearestClimbPose(AllianceColor allianceColor);
};

#endif