#include "Intake.h"

Intake::Intake()
  : wristMotor{WRIST_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    onWristIntakeMotor{ON_WRIST_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    fixedIntakeMotor{FIXED_INTAKE_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    fixedIntakeMotor2{FIXED_INTAKE_MOTOR_PORT_2, rev::CANSparkMax::MotorType::kBrushed},
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

void Intake::SetIntakeMotorSpeed(double FeederPercent_1, double FeederPercent_2)
{
  onWristIntakeMotor.Set(0);
  fixedIntakeMotor.Set(FeederPercent_1);
  fixedIntakeMotor2.Set(FeederPercent_2);
}

void Intake::SetIntakeMotorSpeed(double OverBumperPercent, double FeederPercent_1, double FeederPercent_2)
{
  onWristIntakeMotor.Set(OverBumperPercent);
  fixedIntakeMotor.Set(FeederPercent_1);
  fixedIntakeMotor2.Set(FeederPercent_2);
}

void Intake::IntakeRing()
{
  if(!GetObjectInIntake()){
    SetIntakeMotorSpeed(INTAKE_SPEED_IN*-1);
  }
  else{
    SetIntakeMotorSpeed(0);
  }
}

void Intake::OuttakeRing()
{
  SetIntakeMotorSpeed(INTAKE_SPEED_OUT);
}

void Intake::ShootRing()
{
  SetIntakeMotorSpeed(INTAKE_SPEED_IN);
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

bool Intake::PIDWristToPoint(WristSetting Point){
  if(Point == LOW){
    return PIDWrist(IntakeWrist::WRIST_LOW);
  }
  else if(Point == HIGH){
    return PIDWrist(IntakeWrist::WRIST_HIGH);
  }
  else if (Point == SHOOT){
    return PIDWrist(IntakeWrist::WRIST_SHOOT);
  }
  return false;
}
