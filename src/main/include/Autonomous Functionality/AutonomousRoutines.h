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
    VisionSwerve *swerveDrive;
    Intake *intake;
    FlywheelSystem *flywheel;
    Elevator *ampMech;

    SwerveDriveAutonomousController *swerveDriveController;
    NoteController *noteController;
    AutonomousShootingController *shootingController;
    AutonomousAmpingController *ampingController;

    Timer masterTimer;
    Timer safetyTimer;
    double splineSection = 0;

public:
    AutonomousController(VisionSwerve *swerveDrive_, Intake *intake_, FlywheelSystem *flywheel_, Elevator *ampMech_, SwerveDriveAutonomousController *swerveDriveController_, NoteController *noteController_, AutonomousShootingController *shootingController_, AutonomousAmpingController *ampingController_);

    void SetupAuto(Pose2d startingPose);
    void SetupBlueCenterShootIntake2Shoot();
    void BlueCenterShootIntake2Shoot();
    void SetupBlueLeftShootIntake3Shoot();
    void BlueLeftShootIntake3Shoot();
    void SetupBlueRightShootIntake1Shoot();
    void BlueRightShootIntake1Shoot();
};