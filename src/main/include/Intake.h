#include "Constants/IntakeConstants.h"
#include "Robot.h"

#ifndef INTAKE_H
#define INTAKE_H

class Intake
{

    private:

        rev::CANSparkMax wristMotor;
        rev::CANSparkMax onWristIntakeMotor;
        rev::CANSparkMax fixedIntakeMotor;
        rev::CANSparkMax fixedIntakeMotor2;
        rev::SparkAbsoluteEncoder *magEncoder;
        frc::DigitalInput m_rangeFinder;

        bool PIDWrist(double point);

    public:

        PID m_WristPID;

        enum WristSetting{LOW, HIGH, SHOOT};

        Intake();

        void SetIntakeMotorSpeed(double percent);

        void SetIntakeMotorSpeed(double FeederPercent_1, double FeederPercent_2);
        
        void SetIntakeMotorSpeed(double OverBumperPercent, double FeederPercent_1, double FeederPercent_2);

        void IntakeRing();
        
        void OuttakeRing();

        void ShootRing();

        double GetWristEncoderReading();

        bool GetObjectInIntake();

        void MoveWristPercent(double percent);

        bool PIDWristToPoint(WristSetting Point);

};

#endif