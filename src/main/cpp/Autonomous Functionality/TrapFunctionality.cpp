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

bool AutonomousTrapController::DriveToNearestClimbPose(AllianceColor allianceColor)
{   
    return true;
}