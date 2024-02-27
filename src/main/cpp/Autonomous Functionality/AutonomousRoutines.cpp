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

void AutonomousController::BasicShootIntakeShoot(AllianceColor allianceColor)
{
    SmartDashboard::PutNumber("spline section", splineSection);
    if (splineSection == 0)
    {
        shootingController->BeginAimAndFire(allianceColor);
        splineSection = 0.5;
    }

    if (splineSection == 0.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool shotNote = shootingController->AimAndFire(allianceColor);
        if (safetyTimer.Get() > 1.5_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 2.5_s)
        {
            swerveDriveController->BeginNextTrajectory();
            safetyTimer.Restart();
            splineSection = 1;
        }
    }

    if (splineSection == 1)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool noteInIntake = noteController->IntakeNoteToSelector();
        swerveDriveController->FollowTrajectory(PoseEstimationType::TagBased);

        SmartDashboard::PutBoolean("note in intake", noteInIntake);

        if (noteInIntake || safetyTimer.Get() > 4_s)
        {
            shootingController->BeginAimAndFire(allianceColor);
            intake->SetIntakeMotorSpeed(0);
            swerveDrive->DriveSwervePercent(0,0,0);
            safetyTimer.Restart();
            splineSection = 1.5;
        }
    }

    if (splineSection == 1.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        bool shotNote = shootingController->AimAndFire(allianceColor);

        if (safetyTimer.Get() > 2_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 3_s)
        {
            safetyTimer.Restart();
            splineSection = 1.75;
        }
    }

    if (splineSection == 1.75)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        intake->SetIntakeMotorSpeed(0);
        swerveDrive->DriveSwervePercent(0,0,0);
    }
}

void AutonomousController::SetupBlueCenterShootIntake2Shoot()
{
    SetupAuto(Pose2d(1.39_m, 5.51_m, Rotation2d(180_deg)));
    swerveDriveController->LoadTrajectory("BCTo2");
}

void AutonomousController::BlueCenterShootIntake2Shoot()
{
    BasicShootIntakeShoot(AllianceColor::BLUE);
}

void AutonomousController::SetupBlueLeftShootIntake3Shoot()
{
    SetupAuto(Pose2d(0.76_m, 6.68_m, Rotation2d(-120_deg)));
    swerveDriveController->LoadTrajectory("BLTo3");
}

void AutonomousController::BlueLeftShootIntake3Shoot()
{
    BasicShootIntakeShoot(AllianceColor::BLUE);
}

void AutonomousController::SetupBlueRightShootIntake1Shoot()
{
    SetupAuto(Pose2d(0.74_m, 4.35_m, Rotation2d(120_deg)));
    swerveDriveController->LoadTrajectory("BRTo1");
}

void AutonomousController::BlueRightShootIntake1Shoot()
{
    BasicShootIntakeShoot(AllianceColor::BLUE);
}

void AutonomousController::SetupRedCenterShootIntake10Shoot()
{
    SetupAuto(Pose2d(15.22_m, 5.57_m, Rotation2d(0_deg)));
    swerveDriveController->LoadTrajectory("RCTo10");
}

void AutonomousController::RedCenterShootIntake10Shoot()
{
    BasicShootIntakeShoot(AllianceColor::RED);
}

void AutonomousController::SetupRedLeftShootIntake9Shoot()
{
    SetupAuto(Pose2d(15.81_m, 4.43_m, Rotation2d(60_deg)));
    swerveDriveController->LoadTrajectory("RLTo9");
}

void AutonomousController::RedLeftShootIntake9Shoot()
{
    BasicShootIntakeShoot(AllianceColor::RED);
}

void AutonomousController::SetupRedRightShootIntake11Shoot()
{
    SetupAuto(Pose2d(15.79_m, 6.7_m, Rotation2d(150_deg)));
    swerveDriveController->LoadTrajectory("RRTo11");
}

void AutonomousController::RedRightShootIntake11Shoot()
{
    BasicShootIntakeShoot(AllianceColor::RED);
}

void AutonomousController::SetupBlueLeftShootIntake3ShootIntake8Shoot()
{
    SetupAuto(Pose2d(0.76_m, 6.68_m, Rotation2d(-120_deg)));
    swerveDriveController->LoadTrajectory("BLTo3");
    swerveDriveController->LoadTrajectory("3To8ToShoot");
}

void AutonomousController::BlueLeftShootIntake3ShootIntake8Shoot()
{
    SmartDashboard::PutNumber("spline section", splineSection);
    SmartDashboard::PutNumber("safety timer", safetyTimer.Get().value());

    if (splineSection == 0)
    {
        shootingController->BeginAimAndFire(AllianceColor::BLUE);
        splineSection = 0.5;
    }

    if (splineSection == 0.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool shotNote = shootingController->AimAndFire(AllianceColor::BLUE);
        if (safetyTimer.Get() > 1.5_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 2.5_s)
        {
            swerveDriveController->BeginNextTrajectory();
            safetyTimer.Restart();
            splineSection = 1;
        }
    }

    if (splineSection == 1)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool noteInIntake = noteController->IntakeNoteToSelector();
        swerveDriveController->FollowTrajectory(PoseEstimationType::TagBased);

        if (noteInIntake || safetyTimer.Get() > 4_s)
        {
            shootingController->BeginAimAndFire(AllianceColor::BLUE);
            intake->SetIntakeMotorSpeed(0);
            swerveDrive->DriveSwervePercent(0,0,0);
            safetyTimer.Restart();
            splineSection = 1.5;
        }
    }

    if (splineSection == 1.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        bool shotNote = shootingController->AimAndFire(AllianceColor::BLUE);

        if (safetyTimer.Get() > 3_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 4_s)
        {
            safetyTimer.Restart();
            swerveDriveController->BeginNextTrajectory();
            splineSection = 2;
        }
    }

    if (splineSection == 2)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool noteInIntake = noteController->IntakeNoteToSelector();
        swerveDriveController->FollowTrajectory(PoseEstimationType::TagBased);

        if (noteInIntake || safetyTimer.Get() > 8_s)
        {
            shootingController->BeginAimAndFire(AllianceColor::BLUE);
            intake->SetIntakeMotorSpeed(0);
            swerveDrive->DriveSwervePercent(0,0,0);
            safetyTimer.Restart();
            splineSection = 2.5;
        }
    }

    if (splineSection == 2.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        bool shotNote = shootingController->AimAndFire(AllianceColor::BLUE);

        if (safetyTimer.Get() > 2_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 3_s)
        {
            safetyTimer.Restart();
            splineSection = 2.75;
        }
    }

    if (splineSection == 2.75)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        intake->SetIntakeMotorSpeed(0);
        swerveDrive->DriveSwervePercent(0,0,0);
    }
}

void AutonomousController::SetupBlueLeftShootIntake3ShootIntake8ShootTESTING()
{
    SetupAuto(Pose2d(0.76_m, 6.68_m, Rotation2d(-120_deg)));
    swerveDriveController->LoadTrajectory("BLTo3");
    swerveDriveController->LoadTrajectory("3To8");
    swerveDriveController->LoadTrajectory("8ToShoot");
}

void AutonomousController::BlueLeftShootIntake3ShootIntake8ShootTESTING()
{
    SmartDashboard::PutNumber("spline section", splineSection);
    if (splineSection == 0)
    {
        shootingController->BeginAimAndFire(AllianceColor::BLUE);
        splineSection = 0.5;
    }

    if (splineSection == 0.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool shotNote = shootingController->AimAndFire(AllianceColor::BLUE);
        if (safetyTimer.Get() > 1.5_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 2.5_s)
        {
            swerveDriveController->BeginNextTrajectory();
            safetyTimer.Restart();
            splineSection = 1;
        }
    }

    if (splineSection == 1)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool noteInIntake = noteController->IntakeNoteToSelector();
        swerveDriveController->FollowTrajectory(PoseEstimationType::TagBased);

        if (noteInIntake || safetyTimer.Get() > 4_s)
        {
            shootingController->BeginAimAndFire(AllianceColor::BLUE);
            intake->SetIntakeMotorSpeed(0);
            swerveDrive->DriveSwervePercent(0,0,0);
            safetyTimer.Restart();
            splineSection = 1.5;
        }
    }

    if (splineSection == 1.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        bool shotNote = shootingController->AimAndFire(AllianceColor::BLUE);

        if (safetyTimer.Get() > 2_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 3_s)
        {
            safetyTimer.Restart();
            swerveDriveController->BeginNextTrajectory();
            splineSection = 2;
        }
    }

    if (splineSection == 2)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool trajectoryComplete = swerveDriveController->FollowTrajectory(PoseEstimationType::TagBased);

        if (trajectoryComplete || safetyTimer.Get() > 6_s)
        {
            swerveDriveController->BeginDriveToNote();
            safetyTimer.Restart();
            splineSection = 2.5;
        }
    }

    if (splineSection == 2.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        swerveDriveController->DriveToNote();
        bool done = noteController->IntakeNoteToSelector();

        if (done || safetyTimer.Get() > 3_s)
        {
            swerveDriveController->BeginNextTrajectory();
            safetyTimer.Restart();
            splineSection = 3;
        }
    }

    if (splineSection == 3)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        bool trajectoryComplete = swerveDriveController->FollowTrajectory(PoseEstimationType::TagBased);

        if (trajectoryComplete || safetyTimer.Get() > 6_s)
        {
            shootingController->BeginAimAndFire(AllianceColor::BLUE);
            safetyTimer.Restart();
            splineSection = 3.5;
        }
    }

    if (splineSection == 3.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        bool shotNote = shootingController->AimAndFire(AllianceColor::BLUE);

        if (safetyTimer.Get() > 2_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 3_s)
        {
            safetyTimer.Restart();
            splineSection = 3.75;
        }
    }

    if (splineSection == 3.75)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        intake->SetIntakeMotorSpeed(0);
        swerveDrive->DriveSwervePercent(0,0,0);
    }
}

void AutonomousController::SetupBlueRightShootIntake1ShootIntake2ShootIntake3()
{
    SetupAuto(Pose2d(0.74_m, 4.35_m, Rotation2d(120_deg)));
    swerveDriveController->LoadTrajectory("BRTo1");
    swerveDriveController->LoadTrajectory("1To2");
    swerveDriveController->LoadTrajectory("2To3");
}

void AutonomousController::BlueRightShootIntake1ShootIntake2ShootIntake3()
{
    SmartDashboard::PutNumber("spline section", splineSection);
    SmartDashboard::PutNumber("safety timer", safetyTimer.Get().value());

    if (splineSection == 0)
    {
        shootingController->BeginAimAndFire(AllianceColor::BLUE);
        splineSection = 0.5;
    }

    if (splineSection == 0.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool shotNote = shootingController->AimAndFire(AllianceColor::BLUE);
        if (safetyTimer.Get() > 1.5_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 2.5_s)
        {
            swerveDriveController->BeginNextTrajectory();
            safetyTimer.Restart();
            splineSection = 1;
        }
    }

    if (splineSection == 1)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool noteInIntake = noteController->IntakeNoteToSelector();
        swerveDriveController->FollowTrajectory(PoseEstimationType::TagBased);

        SmartDashboard::PutBoolean("note in intake", noteInIntake);

        if (noteInIntake || safetyTimer.Get() > 4_s)
        {
            shootingController->BeginAimAndFire(AllianceColor::BLUE);
            intake->SetIntakeMotorSpeed(0);
            swerveDrive->DriveSwervePercent(0,0,0);
            safetyTimer.Restart();
            splineSection = 1.5;
        }
    }

    if (splineSection == 1.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        bool shotNote = shootingController->AimAndFire(AllianceColor::BLUE);

        if (safetyTimer.Get() > 2_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 3_s)
        {
            swerveDriveController->BeginNextTrajectory();
            safetyTimer.Restart();
            splineSection = 2;
        }
    }

    if (splineSection == 2)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool noteInIntake = noteController->IntakeNoteToSelector();
        swerveDriveController->FollowTrajectory(PoseEstimationType::TagBased);

        SmartDashboard::PutBoolean("note in intake", noteInIntake);

        if (noteInIntake || safetyTimer.Get() > 4_s)
        {
            shootingController->BeginAimAndFire(AllianceColor::BLUE);
            intake->SetIntakeMotorSpeed(0);
            swerveDrive->DriveSwervePercent(0,0,0);
            safetyTimer.Restart();
            splineSection = 2.5;
        }
    }

    if (splineSection == 2.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        bool shotNote = shootingController->AimAndFire(AllianceColor::BLUE);

        if (safetyTimer.Get() > 2_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 3_s)
        {
            swerveDriveController->BeginNextTrajectory();
            safetyTimer.Restart();
            splineSection = 3;
        }
    }

    if (splineSection == 3)
    {
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
        bool noteInIntake = noteController->IntakeNoteToSelector();
        swerveDriveController->FollowTrajectory(PoseEstimationType::TagBased);

        SmartDashboard::PutBoolean("note in intake", noteInIntake);

        if (noteInIntake || safetyTimer.Get() > 4_s)
        {
            shootingController->BeginAimAndFire(AllianceColor::BLUE);
            intake->SetIntakeMotorSpeed(0);
            swerveDrive->DriveSwervePercent(0,0,0);
            safetyTimer.Restart();
            splineSection = 3.5;
        }
    }

    if (splineSection == 3.5)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        bool shotNote = shootingController->AimAndFire(AllianceColor::BLUE);

        if (safetyTimer.Get() > 2_s)
        {
            intake->ShootNote();
        }

        if (shotNote || safetyTimer.Get() > 3_s)
        {
            safetyTimer.Restart();
            splineSection = 3.75;
        }
    }

    if (splineSection == 3.75)
    {
        intake->PIDWristToPoint(Intake::WristSetting::SHOOT);
        intake->SetIntakeMotorSpeed(0);
        swerveDrive->DriveSwervePercent(0,0,0);
    }
}

void AutonomousController::SetupFastBlueRightShootIntake1ShootIntake2ShootIntake3()
{
    SetupAuto(Pose2d(0.74_m, 4.35_m, Rotation2d(120_deg)));
    swerveDriveController->LoadTrajectory("BRTo1To2To3");
    swerveDriveController->BeginNextTrajectory();
    currentlyShooting = false;
}

void AutonomousController::FastBlueRightShootIntake1ShootIntake2ShootIntake3()
{
    SmartDashboard::PutNumber("spline section", splineSection);
    SmartDashboard::PutNumber("safety timer", safetyTimer.Get().value());
    
    bool noteInIntake = intake->GetObjectInIntake();
    intake->PIDWristToPoint(Intake::WristSetting::LOW);
    double finalSpeeds[3] = {0.0, 0.0, 0.0};

    bool angled = shootingController->AngleFlywheelToSpeaker(AllianceColor::BLUE);
    bool spinning = shootingController->SpinFlywheelForSpeaker(AllianceColor::BLUE);
    bool cleared = shootingController->ClearElevatorForShot();

    SmartDashboard::PutBoolean("currentlyShooting", currentlyShooting);
    SmartDashboard::PutBoolean("noteInIntake", noteInIntake);
    SmartDashboard::PutBoolean("Spinning", spinning);

    if (!currentlyShooting && !noteInIntake)
    {
        noteController->IntakeNoteToSelector();
        swerveDriveController->CalcTrajectoryDriveValues(PoseEstimationType::TagBased, 1, finalSpeeds);
        swerveDrive->DriveSwerveTagOrientedMetersAndRadians(finalSpeeds[0], finalSpeeds[1], finalSpeeds[2]);
    }
    else
    {
        swerveDriveController->CalcTrajectoryDriveValues(PoseEstimationType::TagBased, 0.25, finalSpeeds);
        bool readyToFire = shootingController->TurnToSpeakerWhileDrivingMetersAndRadians(finalSpeeds[0], finalSpeeds[1], AllianceColor::BLUE);    
        SmartDashboard::PutBoolean("Ready to fire", readyToFire);
        if (!currentlyShooting && 
            ((splineSection != 0 && readyToFire && cleared) || (splineSection == 0 && readyToFire && spinning && cleared)))
        {
            splineSection = 1;
            safetyTimer.Restart();
            intake->ShootNote();
            currentlyShooting = true;
        }

        if (currentlyShooting && safetyTimer.Get() > 0.75_s)
        {
            intake->SetIntakeMotorSpeed(0);
            currentlyShooting = false;
        }
    }

}