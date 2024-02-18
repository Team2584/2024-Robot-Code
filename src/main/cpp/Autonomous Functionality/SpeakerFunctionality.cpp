#include "Autonomous Functionality/SpeakerFunctionality.h"

AutonomousShootingController::AutonomousShootingController(SwerveDriveAutonomousController *swerveDrive_, FlywheelSystem *_flyWheel)
{
    swerveDrive = swerveDrive_;
    flyWheel = _flyWheel;
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

    // Determine what our target angle is
    units::meter_t distance = currentPos.Distance(SPEAKER_POSITION.ToTranslation2d());
    Rotation2d targetAngle = Rotation2d(units::radian_t{atan2(SPEAKER_POSITION.Z().value(), distance.value())});

    SmartDashboard::PutNumber("Target Angler Angle", targetAngle.Radians().value());
    return flyWheel->PIDAngler(targetAngle.Radians().value());
}

bool AutonomousShootingController::AimAndFire()
{
    bool turnt = TurnToSpeaker();
    bool angled = AngleFlywheelToSpeaker();

    if (turnt && angled)
    {
        //flyWheel->RunFeederMotor();
        return true;
    }

    return false;
} 