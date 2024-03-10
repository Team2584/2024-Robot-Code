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
    
    NoteController *noteController;
    Elevator *elevator;
    Climb *climb;
    Intake *intake;
    Timer climbTimer;

public:

    AutonomousTrapController(NoteController *noteController_, Elevator *elevator_, Climb *climb_, Intake *intake_);

    bool PrepareClimb();
    bool ClimbToTrap();
    bool ScoreInTrap();

};

#endif