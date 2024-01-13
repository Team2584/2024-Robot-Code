#include "Constants/IntakeConstants.h"
#include "Robot.h"

class Intake
{
public:
    rev::CANSparkMax intakeMotor;

    Intake();

    void SetIntakeMotorSpeed(double percent);
    void IntakeRing();
    void OuttakeRing();
};