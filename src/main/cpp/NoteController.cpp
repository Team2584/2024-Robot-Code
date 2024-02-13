#include "NoteController.h"

NoteController::NoteController(Intake* _intake, FlywheelSystem* _flywheel, Elevator* _elevator)
:   intake(_intake),
    flywheel(_flywheel),
    elevator(_elevator)
{
}

bool NoteController::IntakeNoteToSelector(){
    if(intake->GetObjectInIntake()){
        intake->IntakeNote();
        return false;
    }

    intake->SetIntakeMotorSpeed(0);
    return true;
}

bool NoteController::ToElevator(){
    bool elevatorPrepared = elevator->MoveToHeight(Elevator::ElevatorSetting::OUTTAKE);

    if (!elevatorPrepared)
    {
        intake->SetIntakeMotorSpeed(0);
        elevator->NoteFromSelector();
        return false;
    }

    bool noteInPosition = elevator->GetObjectInMech() && !intake->GetObjectInTunnel();
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

bool NoteController::FromElevatorToSelector(){
    bool elevatorPrepared = elevator->MoveToHeight(Elevator::ElevatorSetting::OUTTAKE);
    if (!elevatorPrepared)
    {
        intake->NoteFromElevator();
        elevator->SetAmpMotorPercent(0);
        return false;
    }

    bool noteInPosition = intake->GetObjectInIntake();
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
    bool noteInPosition = elevator->GetObjectInMech() && !intake->GetObjectInTunnel();
    if (!noteInPosition)
    {
        ToElevator();
        return false;
    }

    intake->SetIntakeMotorSpeed(0);
    elevator->SetAmpMotorPercent(0);
    return elevator->MoveToHeight(position);
}

bool NoteController::ScoreNoteInPosition(Elevator::ElevatorSetting position){
    bool readyToDeposit = LiftNoteToPosition(position);

    if (readyToDeposit)
        elevator->DepositNote();
    
    return false;
}