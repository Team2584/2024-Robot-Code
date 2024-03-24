#include "Intake.h"
#include "FlyWheel.h"
#include "Elevator.h"
#include "Robot.h"

#ifndef NOTECONTROLLER_H
#define NOTECONTROLLER_H

class NoteController
{
    private:

        Intake* intake;
        FlywheelSystem* flywheel;
        Elevator* elevator;

        bool noteBackInSelector = false;
        bool readyToScoreNote = false;
        bool noteFinalPush = false;
        Timer ampTimer;

    public:

        NoteController(Intake* _intake, FlywheelSystem* _flywheel, Elevator* _elevator);

        bool IntakeNoteToSelector();

        void BeginToElevator();
        bool ToElevator();
        void BeginFromElevatorToSelector();
        bool FromElevatorToSelector();

        void BeginLiftNoteToPosition(Elevator::ElevatorSetting position);
        bool LiftNoteToPosition(Elevator::ElevatorSetting position);
        void BeginScoreNoteInPosition(Elevator::ElevatorSetting position);
        bool ScoreNoteInPosition(Elevator::ElevatorSetting position);
};

#endif