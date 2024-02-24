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

void AutonomousShootingController::TurnToSpeakerWhileDriving(double xSpeed, double ySpeed, AllianceColor allianceColor)
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

    swerveDrive->TurnToAngleWhileDriving(xSpeed, ySpeed, targetAngle, PoseEstimationType::TagBased); 
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


    if (distance <= 3_m)
        targetAnglerAngle = 0.995 - 0.144 * distance.value(); //Equation found by testing and getting data
    else
        targetAnglerAngle = 0.55;

    SmartDashboard::PutNumber("Target Angler Angle", targetAnglerAngle);
    return flyWheel->PIDAngler(targetAnglerAngle);
}

bool AutonomousShootingController::SpinFlywheelForSpeaker(AllianceColor allianceColor)
{
    return flyWheel->SetFlywheelVelocity(3000);
}

bool AutonomousShootingController::ClearElevatorForShot()
{
    // The class's variable targetAnglerAngle has the goal angle for the angler in radians.
    // I figured feeding this function where the angler wants to be rather than where it is would be more efficient so we don't have the elevator hop around as the angler goes to it's position or wait to raise before the angler reaches it's position
    
    double angle = targetAnglerAngle;

    if(angle > FlywheelConstants::Angler::BLOCKED_LOW && angle < FlywheelConstants::Angler::BLOCKED_HIGH){
        elevator->PIDElevator(ElevatorConstants::ELEV_TRAP);
                elevator->SetAmpMotorPercent(0);

        return elevator->GetElevatorSetpoint();
    }
    else{
        elevator->StopElevator();
                        elevator->SetAmpMotorPercent(0);

        return true;
    }
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