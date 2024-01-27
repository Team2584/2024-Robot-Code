#include "Constants/IntakeConstants.h"
#include "Robot.h"

class Intake
{
public:

    rev::CANSparkMax intakeMotor;
    rev::CANSparkMax wristMotor;
    rev::CANSparkMax fixedIntakeMotor;
    rev::CANSparkMax fixedIntakeMotor2;
    rev::SparkAbsoluteEncoder *magEncoder;
    frc::Ultrasonic m_rangeFinder;

    PID m_WristPID;

    const units::millimeter_t ULTRASONIC_INTAKE_DIST{100};

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

    //If feed/index motor becomes seperate from intake chain remove this
    rev::CANSparkMax* GetFeedMotor();

};