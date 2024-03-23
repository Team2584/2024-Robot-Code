#include "Autonomous Functionality/TrapFunctionality.h"

AutonomousTrapController::AutonomousTrapController(SwerveDriveAutonomousController *swerveController_, NoteController *noteController_, Elevator *elevator_, Climb *climb_, Intake *intake_)
: swerveController{swerveController_},
noteController{noteController_},
elevator{elevator_},
climb{climb_},
intake{intake_}
{
}

bool AutonomousTrapController::PrepareClimb(){
    climb->ClimbPID(ClimbConstants::MaxHeight);
    elevator->MoveToHeight(Elevator::ElevatorSetting::LOW);
    intake->PIDWristToPoint(Intake::WristSetting::LOW);
    return climb->GetClimbAtPos();
}

bool AutonomousTrapController::AttachHooks(){
    climb->ClimbPID(ClimbConstants::AttatchingHeight);
    elevator->MoveToHeight(Elevator::ElevatorSetting::LOW);
    intake->PIDWristToPoint(Intake::WristSetting::LOW);
    return climb->GetClimbAtPos();
}

bool AutonomousTrapController::ClimbToTrap(){
    climb->ClimbPID(ClimbConstants::MinHeight);
    if (climb->leftEncoder.GetPosition() < 0.2)
        intake->PIDWrist(Intake::WristSetting::SHOOT);
    else
        intake->PIDWristToPoint(Intake::WristSetting::LOW);
    return climb->GetClimbAtPos();
}

bool AutonomousTrapController::ScoreInTrap(){
    return noteController->ScoreNoteInPosition(Elevator::ElevatorSetting::TRAP);
}

bool AutonomousTrapController::LockRotationToNearestClimbPose(AllianceColor allianceColor, double xSpeed, double ySpeed){
    Rotation2d heading = swerveController->swerveDrive->GetOdometryPose().Rotation();
    if (allianceColor == AllianceColor::BLUE)
    {
        if (fabs((heading - Rotation2d(180_deg)).Degrees().value()) <= 60)
            return swerveController->TurnToAngleWhileDriving(xSpeed, ySpeed, Rotation2d(180_deg), PoseEstimationType::PureOdometry);
        else if(fabs((heading - Rotation2d(300_deg)).Degrees().value()) <= 60)   
            return swerveController->TurnToAngleWhileDriving(xSpeed, ySpeed, Rotation2d(300_deg), PoseEstimationType::PureOdometry);
        else
            return swerveController->TurnToAngleWhileDriving(xSpeed, ySpeed, Rotation2d(60_deg), PoseEstimationType::PureOdometry);
    }
    else
    {
        if (fabs((heading - Rotation2d(0_deg)).Degrees().value()) <= 60)
            return swerveController->TurnToAngleWhileDriving(xSpeed, ySpeed, Rotation2d(0_deg), PoseEstimationType::PureOdometry);
        else if(fabs((heading - Rotation2d(120_deg)).Degrees().value()) <= 60)   
            return swerveController->TurnToAngleWhileDriving(xSpeed, ySpeed, Rotation2d(120_deg), PoseEstimationType::PureOdometry);
        else
            return swerveController->TurnToAngleWhileDriving(xSpeed, ySpeed, Rotation2d(240_deg), PoseEstimationType::PureOdometry);
    }
    return false;
}

bool AutonomousTrapController::DriveToNearestClimbPose(AllianceColor allianceColor)
{   
    return true;
}