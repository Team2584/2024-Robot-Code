#include "FlyWheel.h"

FlywheelSystem::FlywheelSystem(Intake * _m_intake)
  : FlywheelMotor1{FLYWHEEL_MOTOR_1, rev::CANSparkFlex::MotorType::kBrushless},
    FlywheelMotor2{FLYWHEEL_MOTOR_2, rev::CANSparkFlex::MotorType::kBrushless},
    FlywheelAnglingMotor{FLYWHEEL_ANGLING_MOTOR, rev::CANSparkFlex::MotorType::kBrushless},
    FlywheelAnglerFF{FlywheelConstants::Angler::KS, FlywheelConstants::Angler::KG, FlywheelConstants::Angler::KV},
    m_intake{_m_intake},
    TopFlywheel{FlywheelSpeedController(&FlywheelMotor1)},
    BottomFlywheel{FlywheelSpeedController(&FlywheelMotor2)},
    FlywheelAnglerPID{FlywheelConstants::Angler::KP, FlywheelConstants::Angler::KI, FlywheelConstants::Angler::KD, FlywheelConstants::Angler::KI_MAX, 
                     FlywheelConstants::Angler::MIN_SPEED, FlywheelConstants::Angler::MAX_SPEED, FlywheelConstants::Angler::POS_TOLERANCE, FlywheelConstants::Angler::VELOCITY_TOLERANCE}
{
    magEncoder = new rev::SparkAbsoluteEncoder(FlywheelAnglingMotor.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle));
}

/**
 * @brief Sets both flywheel motors to a percent
 * @param percent Percent to set both motors to
 * @note Do not use this function in competition, use SetVelocity()
*/
void FlywheelSystem::SpinFlywheelPercent(double percent)
{
  FlywheelMotor1.Set(-percent);
  FlywheelMotor2.Set(-percent);
}

/**
 * @brief Runs feeder motor if object is in intake
*/
void FlywheelSystem::RunFeederMotor()
{
  if ((m_intake->GetObjectInIntake())){
    m_intake->SetIntakeMotorSpeed(0,60);
  }
  else {
    m_intake->SetIntakeMotorSpeed(0,0);
  }
}

/**
 * @brief Sets flywheel object velocities using PID+FeedForward
 * @param velocity Velocity (RPM) to set both flywheels to
 * @returns True if both flywheels are at Setpoint
*/
bool FlywheelSystem::SetFlywheelVelocity(double velocity){
  return SetFlywheelVelocity(velocity, velocity);
}

/**
 * @brief Sets flywheel object velocities using PID+FeedForward
 * @param bottomVelocity Velocity (RPM) to set the bottom flywheel to
 * @param topVelocity Velocity (RPM) to set the top flywheel to
 * @returns True if both flywheels are at Setpoint
*/
bool FlywheelSystem::SetFlywheelVelocity(double bottomVelocity, double topVelocity){
  TopFlywheel.SpinFlyWheelRPM(-topVelocity);
  BottomFlywheel.SpinFlyWheelRPM(-bottomVelocity);
  return (TopFlywheel.AtSetpoint() && BottomFlywheel.AtSetpoint());
}

/**
 * @brief Launch Ring if Flywheel Velocities are at setpoint and there is an object in intake
*/
void FlywheelSystem::ShootRing(){
  /*
  if ((TopFlywheel.AtSetpoint() && BottomFlywheel.AtSetpoint())){
    m_intake->SetFeeding(true);
    m_intake->SetIntakeMotorSpeed(0,60);
  }
  else {
    m_intake->SetFeeding(false);
  }
  */
 
}

double FlywheelSystem::GetAnglerEncoderReading()
{
  double reading = magEncoder->GetPosition();
  if (reading > 0.5)
    reading -= 1;
  return reading * M_PI * 2;
}

void FlywheelSystem::MoveAnglerPercent(double percent)
{
  FlywheelAnglingMotor.Set(percent);
}

bool FlywheelSystem::PIDAngler(double point)
{
  if (point > M_PI / 2 || point < 0)
  {
    FlywheelAnglingMotor.Set(0);
    return false;
  }

  units::volt_t PID = units::volt_t{FlywheelAnglerPID.Calculate(GetAnglerEncoderReading(), point)};
  units::volt_t FF = FlywheelAnglerFF.Calculate(units::radian_t{GetAnglerEncoderReading()}, 0_rad / 1_s);
  SmartDashboard::PutNumber("Angler PID", PID.value());
  SmartDashboard::PutNumber("Angler FF", FF.value());
  FlywheelAnglingMotor.SetVoltage(PID + FF);
  return FlywheelAnglerPID.PIDFinished();
}

/**
 * @brief Single-Motor Flywheel Object Constructor
 * @param FL_motor Pointer to a CANSparkFlex motor controller
*/
FlywheelSpeedController::FlywheelSpeedController(rev::CANSparkFlex *FL_motor)
  :  m_flywheelMotor(FL_motor),
    m_shooterFeedforward(FlywheelConstants::KS, FlywheelConstants::KV),
    m_shooterPID{FlywheelConstants::KP,FlywheelConstants::KI,FlywheelConstants::KD}
{
  m_shooterEncoder =  new rev::SparkRelativeEncoder(m_flywheelMotor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
  m_shooterPID.SetTolerance(FlywheelConstants::kShooterToleranceRPS.value(), FlywheelConstants::kShooterTargetRPS_S.value());
}

/**
  @brief Get Neo Vortex's Current Velocity in RPS (Built-in encoder)
*/
double FlywheelSpeedController::GetMeasurement() {
  return m_shooterEncoder->GetVelocity()/60.0;
}

/**
  @return True if velocity is within tolerance
*/
bool FlywheelSpeedController::AtSetpoint() {
  return m_shooterPID.AtSetpoint();
}

/**
  @brief Sets flywheel to desired speed
  @param setpoint Speed in RPM
*/
void FlywheelSpeedController::SpinFlyWheelRPM(double setpoint){
  units::turns_per_second_t RPS{setpoint/60.0};
  m_shooterPID.SetSetpoint(RPS.value());
  UseOutput(m_shooterPID.Calculate(GetMeasurement()), RPS);
}

/**
  @brief Uses output of PID controller and FeedForward Controller to set flywheel speed
  @param output PID controller output
  @param RPS Speed in RPS, as turns/s unit
*/
void FlywheelSpeedController::UseOutput(double output, units::turns_per_second_t setpointRPS) {
  m_flywheelMotor->SetVoltage(units::volt_t{output} + m_shooterFeedforward.Calculate(setpointRPS));
}
