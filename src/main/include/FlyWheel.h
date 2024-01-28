#include "Robot.h"
#include "Intake.h"
#include "constants/FlywheelConstants.h"

#ifndef FLYWHEEL_H
#define FLYWHEEL_H

class FlywheelSpeedController
{
    public:

        frc::PIDController m_shooterPID;
        rev::CANSparkFlex* m_flywheelMotor;
        frc::SimpleMotorFeedforward<units::turns> m_shooterFeedforward;
        rev::SparkRelativeEncoder* m_shooterEncoder;
        
        FlywheelSpeedController(rev::CANSparkFlex *FL_motor);

        void UseOutput(double output, units::turns_per_second_t setpointRPS);

        double GetMeasurement();

        bool AtSetpoint();

        void SpinFlyWheelRPM(double setpoint);
};

class FlywheelSystem
{
    public:
        rev::CANSparkFlex FlywheelMotor1;
        rev::CANSparkFlex FlywheelMotor2;
        rev::CANSparkMax FlywheelAnglingMotor;

        rev::SparkAbsoluteEncoder *magEncoder;

        FlywheelSpeedController TopFlywheel;
        FlywheelSpeedController BottomFlywheel;
        PID FlywheelAnglerPID;
        ArmFeedforward FlywheelAnglerFF;
        
        Intake * m_intake;

        FlywheelSystem(Intake * _m_intake);

        void SpinFlywheelPercent(double percent);

        void RunFeederMotor();

        bool SetFlywheelVelocity(double velocity);
        
        bool SetFlywheelVelocity(double bottomVelocity, double topVelocity);

        void ShootRing();

        double GetAnglerEncoderReading();

        void MoveAnglerPercent(double percent);

        bool PIDAngler(double point);
};

#endif