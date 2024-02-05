#include "NoteController.h"

NoteController::NoteController(Intake* _intake, FlywheelSystem* _flywheel, Elevator* _elevator)
:   intake(_intake),
    flywheel(_flywheel),
    elevator(_elevator)
{
}

void NoteController::UpdateNotePos(){
    if((intake->GetObjectInIntake() && elevator->GetObjectInTunnel()) || GetNotePos() == REVERSING){
        if(!intake->GetObjectInIntake() && !elevator->GetObjectInTunnel()){
            notePos = NoteController::IN_INTAKE;
        }
        else{
            notePos = NoteController::REVERSING;
        }
        return;
    }
    if(intake->GetObjectInIntake()){
        notePos = NoteController::IN_INTAKE;
    }
    if(elevator->GetObjectInTunnel()){
        notePos = NoteController::IN_TUNNEL;
    }
    if(!elevator->GetObjectInTunnel() && elevator->GetObjectInMech()){
        notePos = NoteController::IN_ELEVATOR;
    }
    if(!intake->GetObjectInIntake() && !elevator->GetObjectInTunnel() && !elevator->GetObjectInMech()){
        notePos = NoteController::NO;
    }
}

NoteController::NotePosition NoteController::GetNotePos(){
    return notePos;
}

void NoteController::IntakeNote(){
    intake->SetIntakeMotorSpeed(INTAKE_SPEED_IN*-1);
}

bool NoteController::IntakeNoteSmart(){
    if(GetNotePos() == NO){
        intake->SetIntakeMotorSpeed(INTAKE_SPEED_IN*-1);
    }
    return (GetNotePos() == IN_INTAKE);
}

bool NoteController::Outtake(){
    intake->SetIntakeMotorSpeed(INTAKE_SPEED_OUT);
}

void NoteController::OuttakeFromElevator(){
    intake->SetIntakeMotorSpeed(60,60);
}

bool NoteController::OuttakeFromElevatorSmart(){
    if(GetNotePos() != NO || GetNotePos() == REVERSING){
        intake->SetIntakeMotorSpeed(60,60);
    }
    return (GetNotePos() == IN_INTAKE);
}

void NoteController::ToFlywheel(){
    intake->SetIntakeMotorSpeed(-60, -60);
}

bool NoteController::ToFlywheelShootSmart(){
    if(flywheel->TopFlywheel.AtSetpoint() && flywheel->BottomFlywheel.AtSetpoint() && GetNotePos() == IN_INTAKE){
        intake->SetIntakeMotorSpeed(-60, -60);
    }
    return (GetNotePos() == NO);
}

void NoteController::ToElevator(){
    intake->SetIntakeMotorSpeed(-60,60); //to elevator
    elevator->SetAmpMotorPercent(60);
}

bool NoteController::ToElevatorSmart(){
    if(GetNotePos() != IN_ELEVATOR){
        intake->SetIntakeMotorSpeed(-60,60); //to elevator
        elevator->SetAmpMotorPercent(60);
    }
    return (GetNotePos() == IN_ELEVATOR);
}

void NoteController::DepositNote(){
    elevator->SetAmpMotorPercent(60);
}

bool NoteController::DepositNoteSmart(Elevator::ElevatorSetting _elevSetHeight){
    if(GetNotePos() == IN_ELEVATOR && elevator->GetElevatorAtSetpoint() && (_elevSetHeight == Elevator::AMP || _elevSetHeight == Elevator::TRAP)){
        elevator->SetAmpMotorPercent(60);
    }
    return (GetNotePos() == NO);
}