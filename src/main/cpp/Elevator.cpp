#include "Elevator.h"


Elevator::Elevator()
:   winchMotor{ELEVATOR_MOTOR_PORT},
    ampMotor{AMP_MECH_PORT, rev::CANSparkMax::MotorType::kBrushless},
    ampMechSensor{ampMotor.GetForwardLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen)},
    m_constraints{ElevatorConstants::kMaxVelocity, ElevatorConstants::kMaxAcceleration},
    m_controller{ElevatorConstants::m_kP, ElevatorConstants::m_kI, ElevatorConstants::m_kD, m_constraints},
    m_feedforward{ElevatorConstants::m_kS, ElevatorConstants::m_kG, ElevatorConstants::m_kV},
    m_timeOfFlight{ElevatorConstants::TimeOfFlight::tofCANID,ElevatorConstants::TimeOfFlight::tofOffset, ElevatorConstants::TimeOfFlight::tofAllowedSigma}
{
    ctre::phoenix6::configs::TalonFXConfiguration toConfigure{};
    toConfigure.CurrentLimits.StatorCurrentLimit = 70.0;
    toConfigure.CurrentLimits.StatorCurrentLimitEnable = true; // And enable it
    winchMotor.GetConfigurator().Apply(toConfigure);
    winchMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    winchMotor.SetPosition(0_tr);
    m_controller.SetTolerance(ElevatorConstants::ALLOWABLE_ERROR_POS);
    ampMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, false);
    ampMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, false);
}

bool Elevator::ZeroElevatorTOF(){
    if(m_timeOfFlight.GetTof() != 0_m){
        winchMotor.SetPosition(m_timeOfFlight.GetTof().value() / ElevatorConstants::ELEV_CONVERSION_FACTOR * -1_tr);
        return true;
    }
    else{
        return false;
    }
}

/**
 * @brief Sets the winch motor's built-in encoer reading to zero
*/
void Elevator::ResetElevatorEncoder()
{
    winchMotor.SetPosition(0_tr);
}

/**
 * @brief Gets the winch motor's built-in encoder reading
*/
double Elevator::GetWinchEncoderReading()
{
    return winchMotor.GetPosition().GetValueAsDouble() * ElevatorConstants::ELEV_CONVERSION_FACTOR * -1;
}

/**
 * @brief Holds the elevator at its current position (Motor on brake)
 * @note This shouldn't be used in driver control (probably), because the feedforward+pid will hold the lift for our application
*/
void Elevator::StopElevator()
{
    MoveElevatorPercent(0);
}
  
/**
 * @brief Sets the winch motor to a percentage value
*/
void Elevator::MoveElevatorPercent(double percent)
{
    winchMotor.Set(percent*-1);
}

void Elevator::BeginPIDElevator(){
    m_controller.Reset(units::meter_t{GetWinchEncoderReading()});
}

/**
 * @brief PIDs Elevator to a setpoint (in meters)
 * @param setpoint Setpoint in meters for the elevator
 * @note The MotionController uses "setpoint" to denote the state target, and "goal" to denote the endpoint
*/
bool Elevator::PIDElevator(double setpoint){
    units::meter_t goal{setpoint};
    m_controller.SetGoal(goal);

    //winchMotor.SetVoltage((units::volt_t{m_controller.Calculate(units::meter_t{winchEncoder->GetPosition()}, goal)} + m_feedforward.Calculate(m_controller.GetSetpoint().velocity))*-1);
    
    units::volt_t PID = units::volt_t{m_controller.Calculate(units::meter_t{GetWinchEncoderReading()}, goal)};
    units::volt_t FF = ElevatorConstants::m_kG;

    if (m_controller.AtGoal())
    {
        PID = 0_V;
        if (setpoint == ElevatorSetting::LOW)
            FF = 0_V;
    }

    if ((PID + FF) < -ElevatorConstants::maxVoltsDown)
        winchMotor.SetVoltage(ElevatorConstants::maxVoltsDown);
    else
        winchMotor.SetVoltage((PID + FF) * -1);

    auto elevv = m_controller.Calculate(units::meter_t{GetWinchEncoderReading()}, goal);
    SmartDashboard::PutNumber("Elev pid out", elevv);
    auto elevf = m_controller.GetSetpoint().velocity;
    SmartDashboard::PutNumber("Elev setpoint", elevf.value());
    auto elevpose = m_controller.GetPositionError();
    SmartDashboard::PutNumber("Elev error", elevpose.value());    
    SmartDashboard::PutBoolean("Elev PID Done", m_controller.AtGoal());
    return m_controller.AtGoal();
}

void Elevator::SetAmpMotorPercent(double percent){
    ampMotor.Set(percent);
}

bool Elevator::GetObjectInMech(){
    return (ampMechSensor.Get());
}

void Elevator::NoteFromSelector(){
    SetAmpMotorPercent(ElevatorConstants::AmpMech::AMP_SPEED_FROM_SELECTOR);
}

void Elevator::NoteToSelector(){
    SetAmpMotorPercent(ElevatorConstants::AmpMech::AMP_SPEED_TO_SELECTOR);
}

void Elevator::DepositNote(){
    SetAmpMotorPercent(ElevatorConstants::AmpMech::AMP_SPEED_DEPOSIT);
}

void Elevator::DepositNoteTrap(){
    SetAmpMotorPercent(ElevatorConstants::AmpMech::TRAP_SPEED_DEPOSIT);
}

bool  Elevator::GetElevatorAtSetpoint(){
    return m_controller.AtGoal();
}

double Elevator::GetElevatorSetpoint(){
    return m_controller.GetGoal().position.value();
}

bool Elevator::MoveToHeight(ElevatorSetting Height) {
    if (Height == AMP){
       return PIDElevator(ElevatorConstants::ELEV_AMP);
    }
    else if (Height == LOW){
        return PIDElevator(ElevatorConstants::ELEV_LOW);
    }
    else if (Height == TRAP){
        return PIDElevator(ElevatorConstants::ELEV_TRAP);
    }
    else if (Height == OUTTAKE){
        return PIDElevator(ElevatorConstants::ELEV_OUTTAKE);
    }
    else if (Height == INTAKE){
        return PIDElevator(ElevatorConstants::ELEV_INTAKE);   
    }
    else if (Height == SOURCEINTAKE){
        return PIDElevator(ElevatorConstants::ELEV_SOURCE);   
    }
    return false;
}
