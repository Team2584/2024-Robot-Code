#include "FlyWheel.h"

FlywheelSystem::FlywheelSystem(rev::CANSparkMax *feed_motor)
  : FlywheelMotor1{FLYWHEEL_MOTOR_1, rev::CANSparkFlex::MotorType::kBrushless},
    FlywheelMotor2{FLYWHEEL_MOTOR_2, rev::CANSparkFlex::MotorType::kBrushless},
    FeedMotor{feed_motor}
{
}

void FlywheelSystem::SetFlywheelMotorSpeed(double percent)
{
  FlywheelMotor1.Set(percent);
  FlywheelMotor2.Set(percent);
}

void FlywheelSystem::FlywheelRing()
{
  SetFlywheelMotorSpeed(FLYWHEEL_BASE_PERCENT);
}
