#include "Autonomous Functionality/AutonomousRoutines.h"

AutonomousController::AutonomousController(VisionSwerve *swerveDrive_, Intake *intake_, FlywheelSystem *flywheel_, Elevator *ampMech_, SwerveDriveAutonomousController *swerveDriveController_, NoteController *noteController_, AutonomousShootingController *shootingController_, AutonomousAmpingController *ampingController_) 
    : masterTimer{}
{
    swerveDrive = swerveDrive_;
    intake = intake_;
    flywheel = flywheel_;
    ampMech = ampMech_;
    swerveDriveController = swerveDriveController_;
    noteController = noteController_;
    shootingController = shootingController_;
    ampingController = ampingController_;
}

void AutonomousController::SetupAuto()
{
    swerveDriveController->ResetTrajectoryQueue();
    splineSection = 0;
    masterTimer.Restart();
}

void AutonomousController::SetupBlueCenterShootIntake2Shoot()
{
    SetupAuto();
    swerveDrive->ResetOdometry(Pose2d(1.39_m, 5.51_m, Rotation2d(180_deg)));
    swerveDrive->ResetTagOdometry(Pose2d(1.39_m, 5.51_m, Rotation2d(180_deg)));
    swerveDriveController->LoadTrajectory("BCTo2");
    swerveDriveController->BeginNextTrajectory();
}

void AutonomousController::BlueCenterShootIntake2Shoot()
{
    if (splineSection == 0)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool shotNote = shootingController->AimAndFire();
        if (shotNote)
            splineSection = 1;
    }

    if (splineSection == 1)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool noteInIntake = noteController->IntakeNoteToSelector();
        bool splineDone = swerveDriveController->FollowTrajectory(PoseEstimationType::TagBased);

        if (noteInIntake)
            splineSection == 1.5;
    }

    if (splineSection == 1.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        bool shotNote = shootingController->AimAndFire();
    }
}