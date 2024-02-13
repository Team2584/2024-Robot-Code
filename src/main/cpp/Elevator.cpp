#include "Elevator.h"


Elevator::Elevator()
:   winchMotor{ELEVATOR_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    ampMotor{AMP_MECH_PORT, rev::CANSparkMax::MotorType::kBrushless},
    ampMechSensor{ampMotor.GetForwardLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen)},
    m_constraints{ElevatorConstants::kMaxVelocity, ElevatorConstants::kMaxAcceleration},
    m_controller{ElevatorConstants::m_kP, ElevatorConstants::m_kI, ElevatorConstants::m_kD, m_constraints},
    m_feedforward{ElevatorConstants::m_kS, ElevatorConstants::m_kG, ElevatorConstants::m_kV}
{
    winchMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    winchEncoder = new rev::SparkRelativeEncoder(winchMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
    winchEncoder->SetPosition(0.0);
    winchEncoder->SetPositionConversionFactor(ElevatorConstants::ELEV_CONVERSION_FACTOR);
    m_controller.SetTolerance(ElevatorConstants::ALLOWABLE_ERROR_POS);
}

/**
 * @brief Sets the winch motor's built-in encoer reading to zero
*/
void Elevator::ResetElevatorEncoder()
{
    winchEncoder->SetPosition(0.0);
}

/**
 * @brief Gets the winch motor's built-in encoder reading
*/
double Elevator::GetWinchEncoderReading()
{
    return winchEncoder->GetPosition();
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

/**
 * @brief PIDs Elevator to a setpoint (in meters)
 * @param setpoint Setpoint in meters for the elevator
 * @note The MotionController uses "setpoint" to denote the state target, and "goal" to denote the endpoint
*/
bool Elevator::PIDElevator(double setpoint){
    units::meter_t goal{setpoint};
    m_controller.SetGoal(goal);

    //winchMotor.SetVoltage((units::volt_t{m_controller.Calculate(units::meter_t{winchEncoder->GetPosition()}, goal)} + m_feedforward.Calculate(m_controller.GetSetpoint().velocity))*-1);
    winchMotor.SetVoltage(units::volt_t{m_controller.Calculate(units::meter_t{winchEncoder->GetPosition()*-1}, goal)} *-1);

    auto elevv = m_controller.Calculate(units::meter_t{winchEncoder->GetPosition()}, goal);
    SmartDashboard::PutNumber("elev pid out", elevv);
    auto elevf = m_controller.GetSetpoint().velocity;
    SmartDashboard::PutNumber("elev setpoint", elevf.value());
    auto elevpose = m_controller.GetPositionError();
    SmartDashboard::PutNumber("elev error", elevpose.value());
    SmartDashboard::PutNumber("elev encoder pos", winchEncoder->GetPosition());

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

bool  Elevator::GetElevatorAtSetpoint(){
    return m_controller.AtGoal();
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
    return false;
}
