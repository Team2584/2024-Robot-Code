#include "Robot.h"
#include "constants/FlywheelConstants.h"

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

        FlywheelSpeedController TopFlywheel;
        FlywheelSpeedController BottomFlywheel;

        Intake * m_intake;

        FlywheelSystem(Intake * _m_intake);

        void SimpleSetFlywheelMotor(double percent);

        void RunFeederMotor();

        bool SetFlywheelVelocity(double velocity);
        
        bool SetFlywheelVelocity(double bottomVelocity, double topVelocity);

        void FlywheelRing();
};