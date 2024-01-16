#include "Constants/IntakeConstants.h"
#include "Robot.h"

class Intake
{
public:

    rev::CANSparkMax intakeMotor;
    rev::CANSparkMax wristMotor;
    rev::CANSparkMax fixedIntakeMotor;
    rev::SparkAbsoluteEncoder *magEncoder;

    double runningWristIntegral = 0;
    double lastWristSpeed = 0;

    Intake();

    void SetIntakeMotorSpeed(double percent);
    void IntakeRing();
    void OuttakeRing();

    double GetWristEncoderReading();

    void MoveWristPercent(double percent);

    bool PIDWrist(double point);

    bool PIDWristDown();

    bool PIDWristUp();

};