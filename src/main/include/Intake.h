#include "Constants/IntakeConstants.h"
#include "Robot.h"

#ifndef INTAKE_H
#define INTAKE_H

class Intake
{
public:

    rev::CANSparkMax intakeMotor;
    rev::CANSparkMax wristMotor;
    rev::CANSparkMax fixedIntakeMotor;
    rev::CANSparkMax fixedIntakeMotor2;
    rev::SparkAbsoluteEncoder *magEncoder;
    frc::DigitalInput m_rangeFinder;

    bool CurrentlyFeeding = false;

    PID m_WristPID;

    Intake();

    void SetIntakeMotorSpeed(double percent);

    void SetIntakeMotorSpeed(double OverBumperPercent, double FeederPercent);

    void IntakeRing();
    
    void OuttakeRing();

    void ShootRing();

    double GetWristEncoderReading();

    bool GetObjectInIntake();

    void MoveWristPercent(double percent);

    bool PIDWrist(double point);

    bool PIDWristDown();

    bool PIDWristUp();

    bool GetFeeding();

    void SetFeeding(bool value);

};

#endif