#include "FlyWheel.h"

Flywheel::Flywheel()
  : FlywheelMotor1{FLYWHEEL_MOTOR_1, rev::CANSparkFlex::MotorType::kBrushless},
    FlywheelMotor2{FLYWHEEL_MOTOR_2, rev::CANSparkFlex::MotorType::kBrushless}
{
}

void Flywheel::SetFlywheelMotorSpeed(double percent)
{
  FlywheelMotor1.Set(percent);
  FlywheelMotor2.Set(percent);
}

void Flywheel::FlywheelRing()
{
  SetFlywheelMotorSpeed(FLYWHEEL_BASE_PERCENT);
}
