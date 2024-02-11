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
        frc::DigitalInput m_tunnelSensor;

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

        bool GetObjectInTunnel();

        void MoveWristPercent(double percent);

        bool PIDWristToPoint(WristSetting Point);

};

#endif