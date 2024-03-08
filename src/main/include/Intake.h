#include "Constants/IntakeConstants.h"
#include "Robot.h"

#ifndef INTAKE_H
#define INTAKE_H

class Intake
{

    private:

        rev::CANSparkMax wristMotor;
        rev::CANSparkFlex onWristIntakeMotor;
        rev::CANSparkMax mainFixedMotor;
        rev::CANSparkMax selectorFixedMotor;
        rev::SparkAbsoluteEncoder *magEncoder;
        frc::DigitalInput m_mainSensor;
        frc::Timer shotTimer;

    public:

        ArmFeedforward m_WristFF;
        PID m_WristPID;

        enum WristSetting{LOW, HIGH, SHOOT};

        Intake();

        void SetIntakeMotorSpeed(double percent);

        void SetIntakeMotorSpeed(double IntakePercent, double SelectorPercent);
        
        void SetIntakeMotorSpeed(double OverBumperPercent, double FixedMotorPercent, double SelectorPercent);

        void IntakeNote();
        void OuttakeNote();

        void BeginShootNote();
        void ShootNote();
        void NoteToElevator();
        void NoteFromElevator();

        double GetWristEncoderReading();

        bool GetObjectInIntake();

        void MoveWristPercent(double percent);
        
        bool PIDWrist(double point);

        bool PIDWristToPoint(WristSetting Point);

};

#endif