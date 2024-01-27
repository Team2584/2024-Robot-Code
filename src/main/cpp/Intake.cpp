#include "Intake.h"

Intake::Intake()
  : intakeMotor{INTAKE_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless}, 
    wristMotor{WRIST_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    fixedIntakeMotor{FIXED_INTAKE_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    m_rangeFinder{1},
    m_WristPID{WRISTKP,WRISTKI,WRISTKD,WRISTKIMAX,WRISTMIN_SPEED,WRISTMAX_SPEED,WRIST_POS_ERROR,WRIST_VELOCITY_ERROR}
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
  MoveWristPercent(m_WristPID.Calculate(GetWristEncoderReading(),point));
  return m_WristPID.PIDFinished();
}

bool Intake::PIDWristDown()
{
  return PIDWrist(WRIST_LOW);
}

bool Intake::PIDWristUp()
{
  return PIDWrist(WRIST_HIGH);
}

bool Intake::GetFeeding(){
  return CurrentlyFeeding;
}

void Intake::SetFeeding(bool value){
  CurrentlyFeeding = value;
}

