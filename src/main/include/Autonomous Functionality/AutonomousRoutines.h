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
    bool currentlyShooting = false;
    units::meter_t maxDistanceShot = 100_m;

public:
    AutonomousController(VisionSwerve *swerveDrive_, Intake *intake_, FlywheelSystem *flywheel_, Elevator *ampMech_, SwerveDriveAutonomousController *swerveDriveController_, NoteController *noteController_, AutonomousShootingController *shootingController_, AutonomousAmpingController *ampingController_);

    void SetupAuto(Pose2d startingPose);
    void SetupBasicShootIntakeShoot(Pose2d startingPose, string TrajectoryName);
    void BasicShootIntakeShoot(AllianceColor allianceColor);
    void SetupBlueLeftShootIntake3ShootIntake8Shoot();
    void BlueLeftShootIntake3ShootIntake8Shoot();
    void SetupBlueLeftShootIntake3ShootIntake8ShootTESTING();
    void BlueLeftShootIntake3ShootIntake8ShootTESTING();
    void SetupSlowBlueRightShootIntake1ShootIntake2ShootIntake3();
    void SlowBlueRightShootIntake1ShootIntake2ShootIntake3();
    void SetupFollowTrajectoryAndShoot(Pose2d startingPose, string trajectoryName, units::meter_t maxDistanceShot_);
    void FollowTrajectoryAndShoot(AllianceColor allianceColor);
    void DropLongShotFollowTrajectoryAndShoot(AllianceColor allianceColor);
    void SetupBlueNoVision4Note();
    void BlueNoVision4Note();
};