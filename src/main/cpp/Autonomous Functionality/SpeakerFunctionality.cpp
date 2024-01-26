#include "Autonomous Functionality/SpeakerFunctionality.h"

AutonomousShootingController::AutonomousShootingController(SwerveDriveAutonomousController *swerveDrive_, FlywheelSystem *_flyWheel)
{
    swerveDrive = swerveDrive_;
    flyWheel = _flyWheel;
}

bool AutonomousShootingController::TurnToSpeaker()
{
    Pose2d currentPose = swerveDrive->GetTagPose();

    // Determine what our target angle is
    Translation2d diff = SPEAKER_POSITION.ToTranslation2d() - currentPose.Translation(); 
    Rotation2d targetAngle = Rotation2d(units::radian_t{atan2(diff.X().value(), diff.Y().value())});

    return swerveDrive->DriveToPose(Pose2d(currentPose.Translation(), targetAngle), PoseEstimationType::TagBased); // Drive to current pose but at the target angle
}

bool AutonomousShootingController::AngleFlywheelToSpeaker()
{   
    Translation2d currentPos = swerveDrive->GetTagPose().Translation();

    // Determine what our target angle is
    units::meter_t distance = currentPos.Distance(SPEAKER_POSITION.ToTranslation2d());
    Rotation2d targetAngle = Rotation2d(units::radian_t{atan2(SPEAKER_POSITION.Z().value(), distance.value())});

    SmartDashboard::PutNumber("Target Angler Angle", targetAngle.Radians().value());
    return false; //flyWheel->PIDAngler(targetAngle.Radians().value());
}

bool AutonomousShootingController::AimAndFire()
{
    bool turnt = TurnToSpeaker();
    bool angled = AngleFlywheelToSpeaker();

    if (turnt && angled)
    {
        flyWheel->SimpleFlywheelRing();
        return true;
    }

    return false;
} 