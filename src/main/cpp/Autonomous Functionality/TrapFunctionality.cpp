#include "Autonomous Functionality/TrapFunctionality.h"

AutonomousTrapController::AutonomousTrapController(NoteController *noteController_, Elevator *elevator_, Climb *climb_, Intake *intake_)
:noteController{noteController_},
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

bool AutonomousTrapController::ClimbToTrap(){
    climb->ClimbPID(ClimbConstants::MinHeight);
    noteController->BeginScoreNoteInPosition(Elevator::ElevatorSetting::TRAP);
    intake->PIDWrist(Intake::WristSetting::SHOOT);
    return climb->GetClimbAtPos();
}

bool AutonomousTrapController::ScoreInTrap(){
    return noteController->ScoreNoteInPosition(Elevator::ElevatorSetting::TRAP);
}