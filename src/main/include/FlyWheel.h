#include "FlywheelConstants.h"
#include "Robot.h"

class Flywheel
{
public:

    rev::CANSparkFlex FlywheelMotor1;
    rev::CANSparkFlex FlywheelMotor2;


    Flywheel();

    void SetFlywheelMotorSpeed(double percent);
    void FlywheelRing();

};