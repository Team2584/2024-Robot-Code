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

        void UseOutput(double output, double setpoint);

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

        rev::CANSparkMax *FeedMotor;
        frc::DigitalInput *m_IntakeSensor;

        bool CurrentlyFeeding = false;

        FlywheelSystem(rev::CANSparkMax *feed_motor, frc::DigitalInput *intakeSensor);

        void SimpleSetFlywheelMotor(double percent);

        void SimpleFlywheelRing();

        bool SetFlywheelVelocity(double velocity);
        
        bool SetFlywheelVelocity(double bottomVelocity, double topVelocity);

        void FlywheelRing();
};