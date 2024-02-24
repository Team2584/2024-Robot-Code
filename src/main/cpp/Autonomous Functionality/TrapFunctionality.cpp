#include "Autonomous Functionality/TrapFunctionality.h"

AutonomousTrapController::AutonomousTrapController(NoteController *noteController_, Elevator *elevator_, Climb *climb_)
:noteController{noteController_},
elevator{elevator_},
climb{climb_}
{
}

bool AutonomousTrapController::PrepareClimb(){
    climb->ClimbPID(ClimbConstants::MaxHeight);
    elevator->MoveToHeight(Elevator::ElevatorSetting::LOW);
    return climb->GetClimbAtPos() && elevator->GetElevatorAtSetpoint();
}

bool AutonomousTrapController::ClimbToTrap(){
    climb->ClimbPID(ClimbConstants::MinHeight);
    noteController->BeginScoreNoteInPosition(Elevator::ElevatorSetting::TRAP);
    return climb->GetClimbDone();
}

bool AutonomousTrapController::ScoreInTrap(){
    noteController->ScoreNoteInPosition(Elevator::ElevatorSetting::TRAP);
}