#include "Autonomous Functionality/SpeakerFunctionality.h"

AutonomousShootingController::AutonomousShootingController(SwerveDriveAutonomousController *swerveDrive_, FlywheelSystem *_flyWheel, Intake *intake_, Elevator *elevator_)
    : shotTimer{}
{
    swerveDrive = swerveDrive_;
    flyWheel = _flyWheel;
    intake = intake_;
    elevator = elevator_;
}

bool AutonomousShootingController::TurnToSpeaker(AllianceColor allianceColor)
{
    Pose2d currentPose = swerveDrive->swerveDrive->GetTagOdometryPose();

    // Determine what our target angle is
    Translation2d diff;
    Translation2d newSpeakerPos;
    if (allianceColor == AllianceColor::BLUE)
    {
        diff = BLUE_SPEAKER_AIM_POSITION.ToTranslation2d() - currentPose.Translation(); 
        SmartDashboard::PutNumber("DIFF X", diff.X().value());
        SmartDashboard::PutNumber("DIFF Y", diff.Y().value());

        if (diff.X() > -1.5_m)
            newSpeakerPos = Translation2d{units::meter_t{lerpVal(-0.5, -1.5, 0.2, 0.05, diff.X().value())}, BLUE_SPEAKER_AIM_POSITION.Y()};

        if (diff.Y() > 3_m)
            newSpeakerPos = Translation2d(BLUE_SPEAKER_AIM_POSITION.X(), 5.85_m);
        else if (diff.Y() > 1_m)
            newSpeakerPos = Translation2d{BLUE_SPEAKER_AIM_POSITION.X(), units::meter_t{lerpVal(1, 3, 5.54, 5.65, diff.Y().value())}};
        else if (diff.Y() < -3_m)
            newSpeakerPos = Translation2d(BLUE_SPEAKER_AIM_POSITION.X(), 5.35_m);
        else if (diff.Y() < -1_m)
            newSpeakerPos = Translation2d{BLUE_SPEAKER_AIM_POSITION.X(), units::meter_t{lerpVal(-1, -3, 5.54, 5.45, diff.Y().value())}};
        else 
            newSpeakerPos = BLUE_SPEAKER_AIM_POSITION.ToTranslation2d();
    }
    else
    {
        diff = RED_SPEAKER_AIM_POSITION.ToTranslation2d() - currentPose.Translation(); 

        if (diff.X() < 1.5_m)
            newSpeakerPos = Translation2d{units::meter_t{lerpVal(-0.5, -1.5, -0.2, -0.05, diff.X().value())}, BLUE_SPEAKER_AIM_POSITION.Y()};

        if (diff.Y() > 3_m)
            newSpeakerPos = Translation2d(RED_SPEAKER_AIM_POSITION.X(), 5.85_m);
        else if (diff.Y() > 1_m)
            newSpeakerPos = Translation2d{RED_SPEAKER_AIM_POSITION.X(), units::meter_t{lerpVal(1, 3, 5.54, 5.65, diff.Y().value())}};
        else if (diff.Y() < -3_m)
            newSpeakerPos = Translation2d(RED_SPEAKER_AIM_POSITION.X(), 5.35_m);
        else if (diff.Y() < -1_m)
            newSpeakerPos = Translation2d{RED_SPEAKER_AIM_POSITION.X(), units::meter_t{lerpVal(-1, -3, 5.54, 5.55, diff.Y().value())}};
        else 
            newSpeakerPos = RED_SPEAKER_AIM_POSITION.ToTranslation2d();
    }    
    diff = newSpeakerPos - currentPose.Translation(); 
    Rotation2d targetAngle = Rotation2d(units::radian_t{atan2(diff.Y().value(), diff.X().value())});

    SmartDashboard::PutNumber("Targe Speaker Swerve Angle", targetAngle.Degrees().value());

    return swerveDrive->DriveToPose(Pose2d(currentPose.Translation(), targetAngle), PoseEstimationType::TagBased); // Drive to current pose but at the target angle
}

bool AutonomousShootingController::TurnToSpeakerWhileDrivingMetersAndRadians(double xSpeed, double ySpeed, AllianceColor allianceColor)
{
    xSpeed = swerveDrive->swerveDrive->VelocityToPercent(xSpeed);
    ySpeed = swerveDrive->swerveDrive->VelocityToPercent(ySpeed);
    return TurnToSpeakerWhileDriving(xSpeed, ySpeed, allianceColor);
}

bool AutonomousShootingController::TurnToSpeakerWhileDriving(double xSpeed, double ySpeed, AllianceColor allianceColor)
{
    Pose2d currentPose = swerveDrive->swerveDrive->GetTagOdometryPose();

    // Determine what our target angle is
   // Determine what our target angle is
    Translation2d diff;
    Translation2d newSpeakerPos;
    if (allianceColor == AllianceColor::BLUE)
    {
        diff = BLUE_SPEAKER_AIM_POSITION.ToTranslation2d() - currentPose.Translation(); 
        SmartDashboard::PutNumber("DIFF X", diff.X().value());
        SmartDashboard::PutNumber("DIFF Y", diff.Y().value());

        if (diff.X() > -1.5_m)
            newSpeakerPos = Translation2d{units::meter_t{lerpVal(-0.5, -1.5, 0.2, 0.05, diff.X().value())}, BLUE_SPEAKER_AIM_POSITION.Y()};

        if (diff.Y() > 3_m)
            newSpeakerPos = Translation2d(BLUE_SPEAKER_AIM_POSITION.X(), 5.85_m);
        else if (diff.Y() > 1_m)
            newSpeakerPos = Translation2d{BLUE_SPEAKER_AIM_POSITION.X(), units::meter_t{lerpVal(1, 3, 5.54, 5.65, diff.Y().value())}};
        else if (diff.Y() < -3_m)
            newSpeakerPos = Translation2d(BLUE_SPEAKER_AIM_POSITION.X(), 5.35_m);
        else if (diff.Y() < -1_m)
            newSpeakerPos = Translation2d{BLUE_SPEAKER_AIM_POSITION.X(), units::meter_t{lerpVal(-1, -3, 5.54, 5.45, diff.Y().value())}};
        else 
            newSpeakerPos = BLUE_SPEAKER_AIM_POSITION.ToTranslation2d();
    }
    else
    {
        diff = RED_SPEAKER_AIM_POSITION.ToTranslation2d() - currentPose.Translation(); 

        if (diff.X() < 1.5_m)
            newSpeakerPos = Translation2d{units::meter_t{lerpVal(-0.5, -1.5, -0.2, -0.05, diff.X().value())}, BLUE_SPEAKER_AIM_POSITION.Y()};

        if (diff.Y() > 3_m)
            newSpeakerPos = Translation2d(RED_SPEAKER_AIM_POSITION.X(), 5.85_m);
        else if (diff.Y() > 1_m)
            newSpeakerPos = Translation2d{RED_SPEAKER_AIM_POSITION.X(), units::meter_t{lerpVal(1, 3, 5.54, 5.65, diff.Y().value())}};
        else if (diff.Y() < -3_m)
            newSpeakerPos = Translation2d(RED_SPEAKER_AIM_POSITION.X(), 5.35_m);
        else if (diff.Y() < -1_m)
            newSpeakerPos = Translation2d{RED_SPEAKER_AIM_POSITION.X(), units::meter_t{lerpVal(-1, -3, 5.54, 5.55, diff.Y().value())}};
        else 
            newSpeakerPos = RED_SPEAKER_AIM_POSITION.ToTranslation2d();
    }    
    diff = newSpeakerPos - currentPose.Translation(); 
    Rotation2d targetAngle = Rotation2d(units::radian_t{atan2(diff.Y().value(), diff.X().value())});

    SmartDashboard::PutNumber("Targe Speaker Swerve Angle", targetAngle.Degrees().value());

    return swerveDrive->TurnToAngleWhileDriving(xSpeed, ySpeed, targetAngle, PoseEstimationType::TagBased); 
} 

bool AutonomousShootingController::AngleFlywheelToSpeaker(AllianceColor allianceColor)
{   
    Translation2d currentPos = swerveDrive->swerveDrive->GetTagOdometryPose().Translation();

    // Determine what our target angle is
    units::meter_t distance;
    
    if (allianceColor == AllianceColor::BLUE)
        distance = currentPos.Distance(BLUE_SPEAKER_POSITION.ToTranslation2d());
    else
        distance = currentPos.Distance(RED_SPEAKER_POSITION.ToTranslation2d());


    if (distance > 6_m)
        targetAnglerAngle = 0.45;
    else if (distance >= 3_m)
        targetAnglerAngle = lerpVal(3, 6, 0.6, 0.45, distance.value());
    else if (distance >= 1.85_m)
        targetAnglerAngle = lerpVal(1.85, 3, 0.68, 0.6, distance.value());
    else
        targetAnglerAngle = lerpVal(1.85, 1.45, 0.68, 0.8, distance.value());

    SmartDashboard::PutNumber("Target Angler Angle", targetAnglerAngle);
    
    return flyWheel->PIDAngler(targetAnglerAngle);
}

bool AutonomousShootingController::SpinFlywheelForSpeaker(AllianceColor allianceColor)
{
    Translation2d currentPos = swerveDrive->swerveDrive->GetTagOdometryPose().Translation();

    // Determine what our target angle is
    units::meter_t distance;
    
    if (allianceColor == AllianceColor::BLUE)
        distance = currentPos.Distance(BLUE_SPEAKER_POSITION.ToTranslation2d());
    else
        distance = currentPos.Distance(RED_SPEAKER_POSITION.ToTranslation2d());
    
    double velocity;
    if (distance > 2_m)
    {
        velocity = lerpVal(2, 3.5, 3500, 5500, distance.value());
        if (velocity > 6000)
            velocity = 6000;
    }
    else
        velocity = 3500;

    SmartDashboard::PutNumber("Target Distance", distance.value());
    SmartDashboard::PutNumber("Target Flywheel Velocity", velocity);
    return flyWheel->SetFlywheelVelocity(velocity);
}

bool AutonomousShootingController::ClearElevatorForShot()
{
    double angle = targetAnglerAngle;

    if(angle > FlywheelConstants::Angler::BLOCKED_LOW && angle < FlywheelConstants::Angler::BLOCKED_HIGH){
        elevator->PIDElevator(0.2);
        elevator->SetAmpMotorPercent(0);
        return elevator->GetElevatorSetpoint();
    }
    else{
        elevator->SetAmpMotorPercent(0);
        return elevator->MoveToHeight(Elevator::ElevatorSetting::LOW);
    }

    return true;
}

void AutonomousShootingController::BeginAimAndFire(AllianceColor allianceColor)
{
    shootingNote = false;
}

bool AutonomousShootingController::AimAndFire(AllianceColor allianceColor)
{
    bool turnt = TurnToSpeaker(allianceColor);
    bool angled = AngleFlywheelToSpeaker(allianceColor);
    bool spinning = SpinFlywheelForSpeaker(allianceColor);
    bool elevatorCleared = ClearElevatorForShot();

    SmartDashboard::PutBoolean("Turned to Speaker", turnt);
    SmartDashboard::PutBoolean("Angled to Speaker", angled);
    SmartDashboard::PutBoolean("Flywheel Spinning Speaker", spinning);
    SmartDashboard::PutBoolean("Elevator Cleared for Speaker", elevatorCleared);


    if (!(turnt && angled && spinning && elevatorCleared))
        return false;
    
    if (!shootingNote)
    {
        shootingNote = true;
        shotTimer.Restart();
        intake->BeginShootNote();
    }

    intake->ShootNote();

    if (shotTimer.Get() < FlywheelConstants::shotTime) 
        return false;

    intake->SetIntakeMotorSpeed(0);
    return true;
} 