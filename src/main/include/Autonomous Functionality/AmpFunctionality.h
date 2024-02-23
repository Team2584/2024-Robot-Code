#include "Robot.h"
#include "Constants/FieldConstants.h"
#include "Constants/ElevatorConstants.h"

#include "NoteController.h"
#include "Autonomous Functionality/SwerveDriveAutoControl.h"

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

    void BeginDriveToAmp();
    bool DriveToAmp();
    void BeginScoreInAmp();
    bool ScoreInAmp();
};