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
    SetIntakeMotorSpeed(INTAKE_SPEED_IN);
}

void Intake::OuttakeRing()
{
    SetIntakeMotorSpeed(INTAKE_SPEED_OUT*-1);
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

  SmartDashboard::PutNumber("Wrist Speed", lastWristSpeed);
  SmartDashboard::PutNumber("Wrist Intended I", intendedI);

  if (lastWristSpeed < WRIST_SPEED_LOW_THRESHHOLD){
    lastWristSpeed = WRIST_SPEED_LOW_THRESHHOLD;
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

