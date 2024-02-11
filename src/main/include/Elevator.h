#include "Robot.h"
#include "constants/ElevatorConstants.h"

#ifndef ELEVATOR_H
#define ELEVATOR_H

class Elevator
{
    private:

        double toflastSpeed = 0;
        double toflastHeight = 0;

        bool lastSensorValue = false;
        int timesPassed = 0;

        rev::CANSparkMax winchMotor;
        rev::SparkRelativeEncoder *winchEncoder;
        rev::CANSparkMax ampMotor;
    
        rev::SparkLimitSwitch ampMechSensor;
        frc::DigitalInput tunnelSensor;

        frc::TrapezoidProfile<units::meters>::Constraints m_constraints;
        frc::ProfiledPIDController<units::meters> m_controller;
        frc::ElevatorFeedforward m_feedforward;

        bool PIDElevator(double setpoint);
        
    public:

        enum ElevatorSetting{LOW, AMP, TRAP};

        Elevator();

        void SetAmpMotorPercent(double percent);

        double GetWinchEncoderReading();

        void StopElevator();

        void MoveElevatorPercent(double percent);

        void ResetElevatorEncoder();

        bool GetObjectInMech();

        bool PrepareNote();

        void DepositNote();

        bool GetElevatorAtSetpoint();

        bool MoveToHeight(ElevatorSetting Height);

};

#endif