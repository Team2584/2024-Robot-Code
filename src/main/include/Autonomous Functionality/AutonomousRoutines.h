#include "Robot.h"
#include "Constants/FieldConstants.h"
#include "Constants/ElevatorConstants.h"

#include "NoteController.h"
#include "Autonomous Functionality/SwerveDriveAutoControl.h"
#include "Autonomous Functionality/AmpFunctionality.h"
#include "Autonomous Functionality/SpeakerFunctionality.h"


class AutonomousController
{
private:
    SwerveDriveAutonomousController *swerveDrive;
    NoteController *noteController;
    AutonomousShootingController *shootingController;
    AutonomousAmpingController *ampingController;

public:
    AutonomousController(SwerveDriveAutonomousController *swerveDrive_, NoteController *noteController_, AutonomousShootingController *shootingController_, AutonomousAmpingController *ampingController_);

    void SetupBlueCenterShootIntake2Shoot();
    void BlueCenterShootIntake2Shoot();
};