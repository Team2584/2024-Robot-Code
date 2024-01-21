#include "constants/FlywheelConstants.h"
#include "Robot.h"

class FlywheelSpeedController : public frc2::PIDSubsystem
{

    public:

        rev::CANSparkFlex FlywheelMotor;
        rev::SparkRelativeEncoder m_shooterEncoder;
        frc::SimpleMotorFeedforward<units::turns> m_shooterFeedforward;

        FlywheelSpeedController();

        void UseOutput(double output, double setpoint) override;

        double GetMeasurement() override;

        bool AtSetpoint();

        void RunFeeder();

        void StopFeeder();

};

class FlywheelSystem
{
    public:
        rev::CANSparkFlex FlywheelMotor1;
        rev::CANSparkFlex FlywheelMotor2;
        rev::CANSparkMax *FeedMotor;

        FlywheelSystem(rev::CANSparkMax *feed_motor);

        void SetFlywheelMotorSpeed(double percent);
        void FlywheelRing();

};