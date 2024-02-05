#include "Intake.h"
#include "FlyWheel.h"
#include "Elevator.h"
#include "Robot.h"

#ifndef NOTECONTROLLER_H
#define NOTECONTROLLER_H

class NoteController
{
    public:

        enum NotePosition{NO, IN_INTAKE, FLYWHEEL, IN_TUNNEL, IN_ELEVATOR, REVERSING};

    private:

        Intake* intake;
        FlywheelSystem* flywheel;
        Elevator* elevator;

        NotePosition notePos = NO;

    public:

        NoteController(Intake* _intake, FlywheelSystem* _flywheel, Elevator* _elevator);

        void UpdateNotePos();
        NotePosition GetNotePos();

        void IntakeToStop();
        bool IntakeToStopSmart();

        bool Outtake();
        void OuttakeFromElevator();
        bool OuttakeFromElevatorSmart();

        void ToFlywheelShoot();
        bool ToFlywheelShootSmart();
        
        void ToElevator();
        bool ToElevatorSmart();

        void DepositNote();
        bool DepositNoteSmart(Elevator::ElevatorSetting _elevSetHeight);
        

};

#endif