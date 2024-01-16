#include "Intake.h"

Intake::Intake()
    : intakeMotor{INTAKE_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless}, wristMotor{WRIST_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless}
{
    magEncoder = new rev::SparkAbsoluteEncoder(wristMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle));
    wristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
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
    SetIntakeMotorSpeed(-1);
}

double Intake::GetWristEncoderReading()
{
  double reading = magEncoder->GetPosition();
  return reading;
}

void Intake::MoveWristPercent(double percent)
{
  wristMotor.Set(percent);
}

bool Intake::PIDWrist(double point)
{
  double error = GetWristEncoderReading() - point;

  SmartDashboard::PutNumber("Wrist Error", error);

  if (fabs(error) < ALLOWABLE_ERROR_WRIST)
  {
    MoveWristPercent(0);
    return true;
  }

  double intendedI = std::clamp(WRISTKI * runningWristIntegral, -1 * WRISTKIMAX, WRISTKIMAX);

  double lastWristSpeed = std::clamp(WRISTKP * error + intendedI, -1 * WRISTMAX_SPEED, WRISTMAX_SPEED);

  SmartDashboard::PutNumber("intended Velocity", lastWristSpeed);
  SmartDashboard::PutNumber("intended I", intendedI);
  SmartDashboard::PutNumber("final speed", lastWristSpeed);

  if (lastWristSpeed < -0.2){
    lastWristSpeed = -0.2;
  }

  MoveWristPercent(lastWristSpeed + WRISTFF);
  return false;
}

bool Intake::PIDWristDown()
{
  return PIDWrist(WRIST_LOW);
}

bool Intake::PIDWristUp()
{
  return PIDWrist(WRIST_HIGH);
}

