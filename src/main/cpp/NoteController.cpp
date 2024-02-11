#include "NoteController.h"

NoteController::NoteController(Intake* _intake, FlywheelSystem* _flywheel, Elevator* _elevator)
:   intake(_intake),
    flywheel(_flywheel),
    elevator(_elevator)
{
}

void NoteController::UpdateNotePos(){
    if(intake->GetObjectInIntake()){
        notePos = NoteController::IN_INTAKE;
    }
    if(notePos == NoteController::IN_INTAKE && !intake->GetObjectInTunnel() && elevator->GetObjectInMech()){
        notePos = NoteController::IN_ELEVATOR;
    }
    if(notePos == NoteController::IN_ELEVATOR && !elevator->GetObjectInMech() && !elevator->GetObjectInMech()){
        notePos = NoteController::NO;
    }
    if(notePos == NoteController::IN_INTAKE && !intake->GetObjectInTunnel()){
        notePos = NoteController::NO;
    }
}

NoteController::NotePosition NoteController::GetNotePos(){
    return notePos;
}

void NoteController::IntakeNote(){
    intake->SetIntakeMotorSpeed(IntakeConstants::INTAKE_SPEED_IN*-1);
}

bool NoteController::IntakeNoteSmart(){
    if(GetNotePos() == NO){
       IntakeNote();
    }
    else{
        intake->SetIntakeMotorSpeed(0);
    }
    return (GetNotePos() == IN_INTAKE);
}

bool NoteController::Outtake(){
    intake->SetIntakeMotorSpeed(IntakeConstants::INTAKE_SPEED_OUT);
}

void NoteController::OuttakeFromElevator(){
    intake->SetIntakeMotorSpeed(60,60);
    elevator->SetAmpMotorPercent(0.75);
}

bool NoteController::OuttakeFromElevatorSmart(){
    if(GetNotePos() != IN_INTAKE ){
        OuttakeFromElevator();
    }
    else{
        elevator->SetAmpMotorPercent(0);
        intake->SetIntakeMotorSpeed(0);
    }
    return (GetNotePos() == IN_INTAKE);
}

void NoteController::ToFlywheelShoot(){
    intake->SetIntakeMotorSpeed(-60, -60);
}

bool NoteController::ToFlywheelShootSmart(){
    if(flywheel->TopFlywheel.AtSetpoint() && flywheel->BottomFlywheel.AtSetpoint() && GetNotePos() == IN_INTAKE){
        ToFlywheelShoot();
    }
    else{
        intake->SetIntakeMotorSpeed(0);
    }
    return (GetNotePos() == NO);
}

void NoteController::ToElevator(){
    intake->SetIntakeMotorSpeed(-60,60); //to elevator
    elevator->SetAmpMotorPercent(60);
}

bool NoteController::ToElevatorSmart(){
    if(GetNotePos() != IN_ELEVATOR){
        ToElevator();
    }
    else{
        elevator->SetAmpMotorPercent(0);
        intake->SetIntakeMotorSpeed(0);
    }
    return (GetNotePos() == IN_ELEVATOR);
}

void NoteController::DepositNote(){
    elevator->SetAmpMotorPercent(-0.75);
}

bool NoteController::DepositNoteSmart(Elevator::ElevatorSetting _elevSetHeight){
    if(GetNotePos() == IN_ELEVATOR && elevator->GetElevatorAtSetpoint() && (_elevSetHeight == Elevator::AMP || _elevSetHeight == Elevator::TRAP)){
        elevator->SetAmpMotorPercent(-0.75);
    }
    else{
        elevator->SetAmpMotorPercent(0);
    }
    return (GetNotePos() == NO);
}