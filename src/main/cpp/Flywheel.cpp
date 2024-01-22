#include "FlyWheel.h"

FlywheelSystem::FlywheelSystem(rev::CANSparkMax *feed_motor)
  : FlywheelMotor1{FLYWHEEL_MOTOR_1, rev::CANSparkFlex::MotorType::kBrushless},
    FlywheelMotor2{FLYWHEEL_MOTOR_2, rev::CANSparkFlex::MotorType::kBrushless},
    FlywheelAnglingMotor{FLYWHEEL_ANGLING_MOTOR, rev::CANSparkFlex::MotorType::kBrushless},
    TopFlywheel{FlywheelSpeedController(&FlywheelMotor1)},
    BottomFlywheel{FlywheelSpeedController(&FlywheelMotor2)},
    FlywheelAnglerPID{ANGLER_KP, ANGLER_KI, ANGLER_KD, ANGLER_KI_MAX, 
                     ANGLER_MIN_SPEED, ANGLER_MAX_SPEED, ANGLER_TOLERANCE, ANGLER_VELOCITY_TOLERANCE},
    FeedMotor{feed_motor}
{
    magEncoder = new rev::SparkAbsoluteEncoder(FlywheelAnglingMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle));
}

void FlywheelSystem::SimpleSetFlywheelMotor(double percent)
{
  FlywheelMotor1.Set(percent);
  FlywheelMotor2.Set(percent);
}

void FlywheelSystem::SimpleFlywheelRing()
{
  SimpleSetFlywheelMotor(FLYWHEEL_BASE_PERCENT);
}

bool FlywheelSystem::SetFlywheelVelocity(double velocity){
  return SetFlywheelVelocity(velocity, velocity);
}

bool FlywheelSystem::SetFlywheelVelocity(double bottomVelocity, double topVelocity){
  TopFlywheel.SpinFlyWheelRPM(topVelocity);
  BottomFlywheel.SpinFlyWheelRPM(bottomVelocity);
  return (TopFlywheel.AtSetpoint() && BottomFlywheel.AtSetpoint());
}

void FlywheelSystem::FlywheelRing(){
  if ((TopFlywheel.AtSetpoint() && BottomFlywheel.AtSetpoint())){
    FeedMotor->Set(kFeederSpeed);
    CurrentlyFeeding = true;
  }
  else {
    CurrentlyFeeding = false;
  }
}

double FlywheelSystem::GetAnglerEncoderReading()
{
  double reading = magEncoder->GetPosition();
  return reading * M_PI * 2;
}

void FlywheelSystem::MoveAnglerPercent(double percent)
{
  FlywheelAnglingMotor.Set(percent);
}

bool FlywheelSystem::PIDAngler(double point)
{
  return FlywheelAnglerPID.Calculate(GetAnglerEncoderReading(), point);
}



FlywheelSpeedController::FlywheelSpeedController(rev::CANSparkFlex *FL_motor)
  : m_shooterPID{frc::PIDController{kP, kI, kD}},
    m_flywheelMotor(FL_motor),
    m_shooterFeedforward(kS, kV) 
{
  m_shooterEncoder =  new rev::SparkRelativeEncoder(m_flywheelMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
  m_shooterPID.SetTolerance(kShooterToleranceRPS.value());
}

double FlywheelSpeedController::GetMeasurement() {
  return m_shooterEncoder->GetVelocity();
}

bool FlywheelSpeedController::AtSetpoint() {
  return m_shooterPID.AtSetpoint();
}

void FlywheelSpeedController::SpinFlyWheelRPM(double setpoint){
  m_shooterPID.SetSetpoint(setpoint);
  UseOutput(m_shooterPID.Calculate(GetMeasurement()), setpoint);
}

void FlywheelSpeedController::UseOutput(double output, double setpoint) {
  units::turns_per_second_t RPS{setpoint/60.0};
  m_flywheelMotor->SetVoltage(units::volt_t{output} + m_shooterFeedforward.Calculate(RPS));
}
