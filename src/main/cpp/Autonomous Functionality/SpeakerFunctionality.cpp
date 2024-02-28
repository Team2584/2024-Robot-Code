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
    
    if (allianceColor == AllianceColor::BLUE)
        diff = BLUE_SPEAKER_AIM_POSITION.ToTranslation2d() - currentPose.Translation(); 
    else
        diff = RED_SPEAKER_AIM_POSITION.ToTranslation2d() - currentPose.Translation(); 
    
    
    Rotation2d targetAngle = Rotation2d(units::radian_t{atan2(diff.Y().value(), diff.X().value())});

    SmartDashboard::PutNumber("Targe Speaker Swerve Angle", targetAngle.Degrees().value());

    return swerveDrive->DriveToPose(Pose2d(currentPose.Translation(), targetAngle), PoseEstimationType::TagBased); // Drive to current pose but at the target angle
}

bool AutonomousShootingController::TurnToSpeakerWhileDriving(double xSpeed, double ySpeed, Translation2d speakerPose)
{
    Pose2d currentPose = swerveDrive->swerveDrive->GetTagOdometryPose();

    // Determine what our target angle is
    Translation2d diff = speakerPose - currentPose.Translation(); 

    Rotation2d targetAngle = Rotation2d(units::radian_t{atan2(diff.Y().value(), diff.X().value())});

    SmartDashboard::PutNumber("Targe Speaker Swerve Angle", targetAngle.Degrees().value());

    return swerveDrive->TurnToAngleWhileDriving(xSpeed, ySpeed, targetAngle, PoseEstimationType::TagBased);   
}

bool AutonomousShootingController::TurnToSpeakerWhileDriving(double xSpeed, double ySpeed, AllianceColor allianceColor)
{    
    if (allianceColor == AllianceColor::BLUE)
        return TurnToSpeakerWhileDriving(xSpeed, ySpeed, BLUE_SPEAKER_AIM_POSITION.ToTranslation2d());
    else
        return TurnToSpeakerWhileDriving(xSpeed, ySpeed, RED_SPEAKER_AIM_POSITION.ToTranslation2d());
}

bool AutonomousShootingController::AngleFlywheelToSpeaker(Translation2d speakerPose)
{
    Translation2d currentPos = swerveDrive->swerveDrive->GetTagOdometryPose().Translation();

    // Determine what our target angle is
    units::meter_t distance = currentPos.Distance(speakerPose);

    if (distance <= 3_m)
        targetAnglerAngle = 1.04 - 0.179* distance.value(); //Equation found by testing and getting data
    else
        targetAnglerAngle = 0.5;

    SmartDashboard::PutNumber("Target Angler Angle", targetAnglerAngle);
    return flyWheel->PIDAngler(targetAnglerAngle);
}

bool AutonomousShootingController::AngleFlywheelToSpeaker(AllianceColor allianceColor)
{   
    if (allianceColor == AllianceColor::BLUE)
        return AngleFlywheelToSpeaker(BLUE_SPEAKER_POSITION.ToTranslation2d());
    else
        return AngleFlywheelToSpeaker(RED_SPEAKER_POSITION.ToTranslation2d());
}

bool AutonomousShootingController::SpinFlywheelForSpeaker(Translation2d speakerPose)
{
    return flyWheel->SetFlywheelVelocity(3300);
}

bool AutonomousShootingController::SpinFlywheelForSpeaker(AllianceColor allianceColor)
{
    return flyWheel->SetFlywheelVelocity(3300);
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
    }

    intake->ShootNote();

    if (shotTimer.Get() < FlywheelConstants::shotTime) 
        return false;

    intake->SetIntakeMotorSpeed(0);
    return true;
} 

units::second_t AutonomousShootingController::TimeForShot(AllianceColor allianceColor)
{
    return 0.5_s;
}

bool AutonomousShootingController::BeginPredictiveShootOnTheMove(AllianceColor allianceColor)
{
    shootingNote = false;
}

bool AutonomousShootingController::PredictiveShootOnTheMove(double xSpeed, double ySpeed, AllianceColor allianceColor)
{
    frc::ChassisSpeeds chassisSpeed = swerveDrive->swerveDrive->GetChassisVelocity();

    if (!shootingNote)
    {
        units::meter_t predictedXChange = chassisSpeed.vx * TimeForShot(allianceColor);
        units::meter_t predictedYChange = chassisSpeed.vy * TimeForShot(allianceColor);
        Translation2d predictedChange = Translation2d(predictedXChange, predictedYChange);

        bool turnt;
        if (allianceColor == AllianceColor::BLUE)
            turnt =  TurnToSpeakerWhileDriving(xSpeed, ySpeed, BLUE_SPEAKER_AIM_POSITION.ToTranslation2d() + predictedChange);
        else
            turnt = TurnToSpeakerWhileDriving(xSpeed, ySpeed, RED_SPEAKER_AIM_POSITION.ToTranslation2d() + predictedChange);

        bool angled;
        if (allianceColor == AllianceColor::BLUE)
            angled = AngleFlywheelToSpeaker(BLUE_SPEAKER_POSITION.ToTranslation2d() + predictedChange);
        else
            angled = AngleFlywheelToSpeaker(RED_SPEAKER_POSITION.ToTranslation2d() + predictedChange);

        bool spinning = SpinFlywheelForSpeaker(allianceColor);
        bool elevatorCleared = ClearElevatorForShot();

        if (!(turnt && angled && spinning && elevatorCleared))
            return false;
        else
        {
            shootingNote = true;
            shotTimer.Restart();
        }
    }
    else
    {
        units::meter_t predictedXChange = chassisSpeed.vx * (TimeForShot(allianceColor) - shotTimer.Get());
        units::meter_t predictedYChange = chassisSpeed.vy * (TimeForShot(allianceColor) - shotTimer.Get());
        Translation2d predictedChange = Translation2d(predictedXChange, predictedYChange);

        if (allianceColor == AllianceColor::BLUE)
            TurnToSpeakerWhileDriving(xSpeed, ySpeed, BLUE_SPEAKER_AIM_POSITION.ToTranslation2d() + predictedChange);
        else
            TurnToSpeakerWhileDriving(xSpeed, ySpeed, RED_SPEAKER_AIM_POSITION.ToTranslation2d() + predictedChange);

        if (allianceColor == AllianceColor::BLUE)
            AngleFlywheelToSpeaker(BLUE_SPEAKER_POSITION.ToTranslation2d() + predictedChange);
        else
            AngleFlywheelToSpeaker(RED_SPEAKER_POSITION.ToTranslation2d() + predictedChange);

        SpinFlywheelForSpeaker(allianceColor);
        ClearElevatorForShot();

        intake->ShootNote();

        if (shotTimer.Get() < FlywheelConstants::shotTime) 
            return false;
        else
        {
            intake->SetIntakeMotorSpeed(0);
            return true;
        }
    }
}