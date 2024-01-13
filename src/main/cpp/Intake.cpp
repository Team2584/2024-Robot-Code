#include "Intake.h"

Intake::Intake()
    : intakeMotor{INTAKE_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless}
{
}

void Intake::SetIntakeMotorSpeed(double percent)
{
    intakeMotor.Set(percent);
}

void Intake::IntakeRing()
{
    SetIntakeMotorSpeed(0.5);
}

void Intake::OuttakeRing()
{
    SetIntakeMotorSpeed(-0.5);
}