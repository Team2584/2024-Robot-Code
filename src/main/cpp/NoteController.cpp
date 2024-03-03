#include "NoteController.h"

NoteController::NoteController(Intake* _intake, FlywheelSystem* _flywheel, Elevator* _elevator)
:   intake(_intake),
    flywheel(_flywheel),
    elevator(_elevator)
{
}

bool NoteController::IntakeNoteToSelector(){
    if(!intake->GetObjectInIntake()){
        intake->IntakeNote();
        return false;
    }

    intake->SetIntakeMotorSpeed(0);
    return true;
}

bool NoteController::ToElevator(){
    bool noteInPosition = elevator->GetObjectInMech();
    if (noteInPosition)
    {        
        intake->SetIntakeMotorSpeed(0);
        elevator->SetAmpMotorPercent(0);
        return true;
    }

    bool elevatorPrepared = elevator->MoveToHeight(Elevator::ElevatorSetting::INTAKE);

    if (!elevatorPrepared)
    {
        intake->SetIntakeMotorSpeed(0);
        elevator->NoteFromSelector();
        return false;
    }

    if (!noteInPosition)
    {
        intake->NoteToElevator();
        elevator->NoteFromSelector();
    }
    else
    {
        intake->SetIntakeMotorSpeed(0);
        elevator->SetAmpMotorPercent(0);
    }

    return noteInPosition;
}

void NoteController::BeginFromElevatorToSelector(){
    noteBackInSelector = false;
}

bool NoteController::FromElevatorToSelector(){
    bool elevatorPrepared = elevator->MoveToHeight(Elevator::ElevatorSetting::OUTTAKE);
    if (!elevatorPrepared)
    {
        intake->SetIntakeMotorSpeed(0);
        elevator->SetAmpMotorPercent(0);
        return false;
    }

    if (intake->GetObjectInIntake() && !noteBackInSelector)
        noteBackInSelector = true;

    bool noteInPosition = noteBackInSelector && !intake->GetObjectInIntake();
    if (!noteInPosition)
    {
        intake->NoteFromElevator();
        elevator->NoteToSelector();
    }
    else
    {
        intake->SetIntakeMotorSpeed(0);
        elevator->SetAmpMotorPercent(0);
    }

    return noteInPosition;
}

bool NoteController::LiftNoteToPosition(Elevator::ElevatorSetting position){
    bool noteInPosition = elevator->GetObjectInMech();
    if (!noteInPosition)
    {
        ToElevator();
        return false;
    }

    intake->SetIntakeMotorSpeed(0);
    elevator->SetAmpMotorPercent(0);
    return elevator->MoveToHeight(position);
}

/**
* Called once before Score note in position (i.e. if score note in position is mapped to a button this function would be called on button press)
*/
void NoteController::BeginScoreNoteInPosition(Elevator::ElevatorSetting position){
    readyToScoreNote = false; 
}

bool NoteController::ScoreNoteInPosition(Elevator::ElevatorSetting position){
    if (!readyToScoreNote)
    {
        readyToScoreNote = LiftNoteToPosition(position);
        return false;
    }
    
    bool noteDeposited = !elevator->GetObjectInMech();
    elevator->MoveToHeight(position);

    if (!noteDeposited)
        elevator->DepositNote();
    else
        elevator->SetAmpMotorPercent(0);

    return noteDeposited;
}