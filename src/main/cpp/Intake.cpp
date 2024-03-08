#include "Intake.h"

Intake::Intake()
  : wristMotor{WRIST_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    onWristIntakeMotor{ON_WRIST_MOTOR_PORT, rev::CANSparkFlex::MotorType::kBrushless},
    mainFixedMotor{MAIN_FIXED_INTAKE_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    selectorFixedMotor{SELECTOR_FIXED_INTAKE_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    m_mainSensor{INTAKE_IR_SENSOR_PORT},
    m_WristFF{IntakeConstants::Wrist::KS, IntakeConstants::Wrist::KG, IntakeConstants::Wrist::KV},
    m_WristPID{IntakeConstants::Wrist::KP,IntakeConstants::Wrist::KI,IntakeConstants::Wrist::KD,IntakeConstants::Wrist::KIMAX,IntakeConstants::Wrist::MIN_SPEED,IntakeConstants::Wrist::MAX_SPEED,IntakeConstants::Wrist::POS_ERROR,IntakeConstants::Wrist::VELOCITY_ERROR},
    shotTimer{}
{
  magEncoder = new rev::SparkAbsoluteEncoder(wristMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle));
  wristMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_WristPID.EnableContinuousInput(0, 1);
  wristMotor.SetInverted(true);
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
  SetIntakeMotorSpeed(IntakeConstants::INTAKE_SPEED_IN, IntakeConstants::FIXED_INTAKE_SPEED_IN, 0);
}

void Intake::OuttakeNote()
{
  SetIntakeMotorSpeed(IntakeConstants::INTAKE_SPEED_OUT, IntakeConstants::FIXED_INTAKE_SPEED_OUT, IntakeConstants::SELECTOR_SPEED_SHOOTER);
}

void Intake::BeginShootNote()
{
  shotTimer.Restart();
  SetIntakeMotorSpeed(0, 0, IntakeConstants::SELECTOR_SPEED_SHOOTER);
}

void Intake::ShootNote()
{
  if (shotTimer.Get() < IntakeConstants::SHOT_INTAKE_TIME)
    SetIntakeMotorSpeed(0, 0, IntakeConstants::SELECTOR_SPEED_SHOOTER);
  else
    SetIntakeMotorSpeed(0, IntakeConstants::INTAKE_SPEED_SHOOTER, IntakeConstants::SELECTOR_SPEED_SHOOTER);
}

void Intake::NoteToElevator()
{
  SetIntakeMotorSpeed(0, IntakeConstants::FIXED_INTAKE_SPEED_IN, IntakeConstants::SELECTOR_SPEED_ELEVATOR);
}

void Intake::NoteFromElevator()
{
  SetIntakeMotorSpeed(0, IntakeConstants::INTAKE_SPEED_BACK_TO_SELECTOR, IntakeConstants::SELECTOR_SPEED_SHOOTER);
}

double Intake::GetWristEncoderReading()
{
  double reading = magEncoder->GetPosition();
  return reading;
}

bool Intake::GetObjectInIntake(){
  return (!m_mainSensor.Get());
}

void Intake::MoveWristPercent(double percent)
{
  wristMotor.Set(percent);
}

bool Intake::PIDWrist(double point)
{  
  units::volt_t PID = units::volt_t{m_WristPID.Calculate(GetWristEncoderReading(), point)};
  units::volt_t FF = m_WristFF.Calculate(units::radian_t{GetWristEncoderReading() * 2 * M_PI - 0.2617}, 0_rad / 1_s);

  // If wrist is down, stop feedforward
  if (point == IntakeConstants::Wrist::WRIST_LOW && m_WristPID.PIDFinished())
  {
    FF = 0_V;
  }

  SmartDashboard::PutNumber("Wrist PID", PID.value());
  SmartDashboard::PutNumber("Wrist FF", FF.value());
  SmartDashboard::PutBoolean("Wrist PID Complete",  m_WristPID.PIDFinished());

  wristMotor.SetVoltage((PID + FF));
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
