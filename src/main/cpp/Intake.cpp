#include "Intake.h"

Intake::Intake()
  : wristMotor{WRIST_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    onWristIntakeMotor{ON_WRIST_MOTOR_PORT, rev::CANSparkFlex::MotorType::kBrushless},
    mainFixedMotor{MAIN_FIXED_INTAKE_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    selectorFixedMotor{SELECTOR_FIXED_INTAKE_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    m_WristPID{IntakeConstants::Wrist::KP,IntakeConstants::Wrist::KI,IntakeConstants::Wrist::KD,IntakeConstants::Wrist::KIMAX,IntakeConstants::Wrist::MIN_SPEED,IntakeConstants::Wrist::MAX_SPEED,IntakeConstants::Wrist::POS_ERROR,IntakeConstants::Wrist::VELOCITY_ERROR},
    m_mainSensor{9},
    m_tunnelSensor{8}
{
  magEncoder = new rev::SparkAbsoluteEncoder(wristMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle));
  wristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Intake::SetIntakeMotorSpeed(double percent)
{
  onWristIntakeMotor.Set(percent);
  mainFixedMotor.Set(percent);
  selectorFixedMotor.Set(percent);
}

void Intake::SetIntakeMotorSpeed(double mainMotorsPercent, double selectorMotorPct)
{
  onWristIntakeMotor.Set(mainMotorsPercent);
  mainFixedMotor.Set(mainMotorsPercent);
  selectorFixedMotor.Set(selectorMotorPct);
}

void Intake::SetIntakeMotorSpeed(double OverBumperPercent, double fixedMotorPercent, double selectorMotorPct)
{
  onWristIntakeMotor.Set(OverBumperPercent);
  mainFixedMotor.Set(fixedMotorPercent);
  selectorFixedMotor.Set(selectorMotorPct);
}

void Intake::IntakeNote()
{
  SetIntakeMotorSpeed(IntakeConstants::INTAKE_SPEED_IN, 0);
}

void Intake::OuttakeNote()
{
  SetIntakeMotorSpeed(IntakeConstants::INTAKE_SPEED_OUT, IntakeConstants::SELECTOR_SPEED_ELEVATOR);
}

void Intake::ShootNote()
{
  SetIntakeMotorSpeed(IntakeConstants::INTAKE_SPEED_IN, IntakeConstants::SELECTOR_SPEED_SHOOTER);
}

void Intake::NoteToElevator()
{
  SetIntakeMotorSpeed(IntakeConstants::INTAKE_SPEED_IN, IntakeConstants::SELECTOR_SPEED_ELEVATOR);
}

void Intake::NoteFromElevator()
{
  SetIntakeMotorSpeed(IntakeConstants::INTAKE_SPEED_OUT, IntakeConstants::SELECTOR_SPEED_SHOOTER);
}

double Intake::GetWristEncoderReading()
{
  double reading = magEncoder->GetPosition();
  return reading;
}

bool Intake::GetObjectInIntake(){
  return (!m_mainSensor.Get());
}

bool Intake::GetObjectInTunnel(){
  return (!m_tunnelSensor.Get());
}

void Intake::MoveWristPercent(double percent)
{
  wristMotor.Set(percent);
}

bool Intake::PIDWrist(double point)
{
  MoveWristPercent(m_WristPID.Calculate(GetWristEncoderReading(),point)*-1);
  return m_WristPID.PIDFinished();
}

bool Intake::PIDWristToPoint(WristSetting Point){
  if(Point == LOW){
    return PIDWrist(IntakeConstants::Wrist::WRIST_LOW);
  }
  else if(Point == HIGH){
    return PIDWrist(IntakeConstants::Wrist::WRIST_HIGH);
  }
  else if (Point == SHOOT){
    return PIDWrist(IntakeConstants::Wrist::WRIST_SHOOT);
  }
  return false;
}
