#include "Robot.h"
#include "constants/FlywheelConstants.h"

#ifndef FLYWHEEL_H
#define FLYWHEEL_H

class FlywheelSpeedController
{
    private:

        frc::SimpleMotorFeedforward<units::turns> m_shooterFeedforward;
        rev::SparkRelativeEncoder* m_shooterEncoder;

        void UseOutput(double output, units::turns_per_second_t setpointRPS);

    public:
        rev::CANSparkFlex* m_flywheelMotor;

        frc::PIDController m_shooterPID;
       
        FlywheelSpeedController(rev::CANSparkFlex *FL_motor);

        double GetMeasurement();

        bool AtSetpoint();

        void SpinFlyWheelRPM(double setpoint);
};

class FlywheelSystem
{

    private:

        rev::CANSparkFlex FlywheelMotor1;
        rev::CANSparkFlex FlywheelMotor2;
        
        ArmFeedforward FlywheelAnglerFF;
        
    public:
        rev::CANSparkFlex FlywheelAnglingMotor;

        frc::DutyCycleEncoder magEncoder;

        FlywheelSpeedController TopFlywheel;
        FlywheelSpeedController BottomFlywheel;
        PID FlywheelAnglerPID;
        
        FlywheelSystem();

        void SpinFlywheelPercent(double percent);

        bool SetFlywheelVelocity(double velocity);
        
        bool SetFlywheelVelocity(double bottomVelocity, double topVelocity);

        double GetAnglerEncoderReading();

        void MoveAnglerPercent(double percent);

        bool PIDAngler(double point);
};

#endif