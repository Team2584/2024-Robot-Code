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
    return climb->GetClimbAtPos();
}

bool AutonomousTrapController::ClimbToTrap(){
    climb->ClimbPID(0);
    noteController->BeginScoreNoteInPosition(Elevator::ElevatorSetting::TRAP);
    return climb->GetClimbAtPos();
}

bool AutonomousTrapController::ScoreInTrap(){
    return noteController->ScoreNoteInPosition(Elevator::ElevatorSetting::TRAP);
}