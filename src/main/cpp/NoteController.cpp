#include "NoteController.h"

NoteController::NoteController(Intake* _intake, FlywheelSystem* _flywheel, Elevator* _elevator)
:   intake(_intake),
    flywheel(_flywheel),
    elevator(_elevator),
    ampTimer{}
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

void NoteController::BeginToElevator(){
    noteFinalPush = false;
}

bool NoteController::ToElevator(){
    bool noteInPosition = elevator->GetObjectInMech();
    if (noteInPosition && noteFinalPush && ampTimer.Get() > ElevatorConstants::AmpMech::AMP_TIME)
    {
        intake->SetIntakeMotorSpeed(0);
        elevator->SetAmpMotorPercent(0);
        return true;
    }
    else if (noteInPosition && noteFinalPush)
    {  
        intake->NoteToElevator();
        elevator->NoteFromSelector();
        return false;
    }
    else if (noteInPosition && !noteFinalPush)
    {
        ampTimer.Restart();
        noteFinalPush = true;
        intake->NoteToElevator();
        elevator->NoteFromSelector();
        return false;   
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
    SmartDashboard::PutNumber("note back in selector", noteBackInSelector);
    SmartDashboard::PutNumber("note in position", noteInPosition);
    
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

void NoteController::BeginLiftNoteToPosition(Elevator::ElevatorSetting position){
    BeginToElevator();
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
    BeginLiftNoteToPosition(position);
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
    {
        if (position == Elevator::ElevatorSetting::TRAP)
            elevator->DepositNoteTrap();
        else
            elevator->DepositNote();
    }
    else
        elevator->SetAmpMotorPercent(0);

    return noteDeposited;
}