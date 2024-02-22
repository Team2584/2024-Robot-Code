#include "Autonomous Functionality/SpeakerFunctionality.h"

AutonomousShootingController::AutonomousShootingController(SwerveDriveAutonomousController *swerveDrive_, FlywheelSystem *_flyWheel, Intake *intake_)
    : shotTimer{}
{
    swerveDrive = swerveDrive_;
    flyWheel = _flyWheel;
    intake = intake_;
}

bool AutonomousShootingController::TurnToSpeaker()
{
    Pose2d currentPose = swerveDrive->swerveDrive->GetTagOdometryPose();

    // Determine what our target angle is
    Translation2d diff = SPEAKER_POSITION.ToTranslation2d() - currentPose.Translation(); 
    Rotation2d targetAngle = Rotation2d(units::radian_t{atan2(diff.Y().value(), diff.X().value())});

    SmartDashboard::PutNumber("Targe Speaker Swerve Angle", targetAngle.Degrees().value());

    return swerveDrive->DriveToPose(Pose2d(currentPose.Translation(), targetAngle), PoseEstimationType::TagBased); // Drive to current pose but at the target angle
}

void AutonomousShootingController::TurnToSpeakerWhileDriving(double xSpeed, double ySpeed)
{
    Pose2d currentPose = swerveDrive->swerveDrive->GetTagOdometryPose();

    // Determine what our target angle is
    Translation2d diff = SPEAKER_POSITION.ToTranslation2d() - currentPose.Translation(); 
    Rotation2d targetAngle = Rotation2d(units::radian_t{atan2(diff.Y().value(), diff.X().value())});

    SmartDashboard::PutNumber("Targe Speaker Swerve Angle", targetAngle.Degrees().value());

    swerveDrive->TurnToAngleWhileDriving(xSpeed, ySpeed, targetAngle, PoseEstimationType::TagBased); 
}

bool AutonomousShootingController::AngleFlywheelToSpeaker()
{   
    Translation2d currentPos = swerveDrive->swerveDrive->GetTagOdometryPose().Translation();
    double targetAngle;

    // Determine what our target angle is
    units::meter_t distance = currentPos.Distance(SPEAKER_POSITION.ToTranslation2d());
    
    if (distance <= 3_m)
        targetAngle = 0.995 - 0.148 * distance.value(); //Equation found by testing and getting data
    else
        targetAngle = 0.55;

    SmartDashboard::PutNumber("Target Angler Angle", targetAngle);
    return flyWheel->PIDAngler(targetAngle);
}

bool AutonomousShootingController::SpinFlywheelForSpeaker()
{
    return flyWheel->SetFlywheelVelocity(3500);
}

bool AutonomousShootingController::AimAndFire()
{
    bool turnt = TurnToSpeaker();
    bool angled = AngleFlywheelToSpeaker();
    bool spinning = SpinFlywheelForSpeaker();

    SmartDashboard::PutBoolean("Turned to Speaker", turnt);
    SmartDashboard::PutBoolean("Angled to Speaker", angled);
    SmartDashboard::PutBoolean("Flyhwheel Spinning Speaker", spinning);


    if (!(turnt && angled && spinning))
        return false;
    
    shotTimer.Restart();
    intake->ShootNote();

    if (shotTimer.Get() < FlywheelConstants::shotTime) 
        return false;

    intake->SetIntakeMotorSpeed(0);
    return true;
} 