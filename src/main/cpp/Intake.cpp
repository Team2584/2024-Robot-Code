#include "Intake.h"

Intake::Intake()
  : intakeMotor{INTAKE_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless}, 
    wristMotor{WRIST_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    fixedIntakeMotor{FIXED_INTAKE_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    m_rangeFinder{1},
    m_WristPID{WRISTKP,WRISTKI,WRISTKD}
{
  magEncoder = new rev::SparkAbsoluteEncoder(wristMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle));
  wristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}

void Intake::SetIntakeMotorSpeed(double percent)
{
  SetIntakeMotorSpeed(percent, percent);
}

void Intake::SetIntakeMotorSpeed(double OverBumperPercent, double FeederPercent)
{
  intakeMotor.Set(OverBumperPercent);
  fixedIntakeMotor.Set(FeederPercent);
}

void Intake::IntakeRing()
{
  if(!GetObjectInIntake()){
    SetIntakeMotorSpeed(INTAKE_SPEED_IN*-1);
  }
}

void Intake::OuttakeRing()
{
  SetIntakeMotorSpeed(INTAKE_SPEED_OUT);
}

double Intake::GetWristEncoderReading()
{
  double reading = magEncoder->GetPosition();
  return reading;
}

bool Intake::GetObjectInIntake(){
  return (!m_rangeFinder.Get());
}

void Intake::MoveWristPercent(double percent)
{
  wristMotor.Set(percent);
}

bool Intake::PIDWrist(double point)
{
  double error = GetWristEncoderReading() - point;

  SmartDashboard::PutNumber("Wrist Error", error);

  if (fabs(error) < WRIST_POS_ERROR)
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

//If feed/index motor becomes seperate from intake chain remove this
rev::CANSparkMax* Intake::GetFeedMotor(){
  return &fixedIntakeMotor;
}

