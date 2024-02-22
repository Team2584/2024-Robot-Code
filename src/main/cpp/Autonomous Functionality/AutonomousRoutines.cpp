#include "Autonomous Functionality/AutonomousRoutines.h"

AutonomousController::AutonomousController(VisionSwerve *swerveDrive_, Intake *intake_, FlywheelSystem *flywheel_, Elevator *ampMech_, SwerveDriveAutonomousController *swerveDriveController_, NoteController *noteController_, AutonomousShootingController *shootingController_, AutonomousAmpingController *ampingController_) 
    : masterTimer{},
      safetyTimer{}
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

void AutonomousController::SetupAuto(Pose2d startingPose)
{
    swerveDrive->ResetOdometry(startingPose);
    swerveDrive->ResetTagOdometry(startingPose);
    swerveDriveController->ResetTrajectoryQueue();
    splineSection = 0;
    masterTimer.Restart();
    safetyTimer.Restart();
}

void AutonomousController::SetupBlueCenterShootIntake2Shoot()
{
    SetupAuto(Pose2d(1.39_m, 5.51_m, Rotation2d(180_deg)));
    swerveDriveController->LoadTrajectory("BCTo2");
    swerveDriveController->BeginNextTrajectory();
}

void AutonomousController::BlueCenterShootIntake2Shoot()
{
    if (splineSection == 0)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool shotNote = shootingController->AimAndFire();
        if (safetyTimer.Get() > 1.5_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 2.5_s)
        {
            safetyTimer.Restart();
            splineSection = 1;
        }
    }

    if (splineSection == 1)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool noteInIntake = noteController->IntakeNoteToSelector();
        bool splineDone = swerveDriveController->FollowTrajectory(PoseEstimationType::TagBased);

        if (noteInIntake || safetyTimer.Get() > 4_s)
        {
            intake->SetIntakeMotorSpeed(0);
            swerveDrive->DriveSwervePercent(0,0,0);
            safetyTimer.Restart();
            splineSection == 1.5;
        }
    }

    if (splineSection == 1.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        bool shotNote = shootingController->AimAndFire();

        if (safetyTimer.Get() > 2_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 3_s)
        {
            safetyTimer.Restart();
            splineSection == 1.75;
        }
    }

    if (splineSection == 1.75)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        intake->SetIntakeMotorSpeed(0);
        swerveDrive->DriveSwervePercent(0,0,0);
    }
}

void AutonomousController::SetupBlueLeftShootIntake3Shoot()
{
    SetupAuto(Pose2d(0.74_m, 4.35_m, Rotation2d(-57.72_deg)));
    swerveDriveController->LoadTrajectory("BLTo3");
    swerveDriveController->BeginNextTrajectory();
}

void AutonomousController::BlueLeftShootIntake3Shoot()
{
    BlueCenterShootIntake2Shoot();
}

void AutonomousController::SetupBlueRightShootIntake1Shoot()
{
    SetupAuto(Pose2d(0.76_m, 6.68_m, Rotation2d(58.39_deg)));
    swerveDriveController->LoadTrajectory("BRTo1");
    swerveDriveController->BeginNextTrajectory();
}

void AutonomousController::BlueRightShootIntake1Shoot()
{
    BlueCenterShootIntake2Shoot();
}