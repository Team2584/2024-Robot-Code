#include "Elevator.h"


Elevator::Elevator()
:   winchMotor{ELEVATOR_MOTOR_PORT, rev::CANSparkMax::MotorType::kBrushless},
    ampMotor{AMP_MECH_PORT, rev::CANSparkMax::MotorType::kBrushless},
    ampMechSensor{ampMotor.GetForwardLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen)},
    m_constraints{e_kMaxVelocity, e_kMaxAcceleration},
    m_controller{e_kP, e_kI, e_kD, m_constraints},
    m_feedforward{e_kS, e_kG, e_kV}
{
    winchMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    winchEncoder = new rev::SparkRelativeEncoder(winchMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));
    winchEncoder->SetPosition(0.0);
    winchEncoder->SetPositionConversionFactor(ELEV_CONVERSION_FACTOR);
    m_controller.SetTolerance(ALLOWABLE_ERROR_ELEV_POS);
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
    winchMotor.Set(percent);
}

/**
 * @brief PIDs Elevator to a setpoint (in meters)
 * @param setpoint Setpoint in meters for the elevator
 * @note The MotionController uses "setpoint" to denote the state target, and "goal" to denote the endpoint
*/
bool Elevator::PIDElevator(double setpoint){
    units::meter_t goal{setpoint};
    m_controller.SetGoal(goal);

    winchMotor.SetVoltage(
        units::volt_t{m_controller.Calculate(units::meter_t{winchEncoder->GetPosition()})} +
        m_feedforward.Calculate(m_controller.GetSetpoint().velocity));

    return m_controller.AtGoal();
}

void Elevator::SetAmpMotorPercent(double percent){
    ampMotor.Set(percent);
}

bool Elevator::GetObjectInMech(){
    return (!ampMechSensor.Get());
}

bool  Elevator::GetElevatorAtSetpoint(){
    return m_controller.AtGoal();
}

bool Elevator::MoveToHeight(ElevatorSetting Height) {
    if (Height == AMP){
       return PIDElevator(ELEV_HIGH);
    }
    else if (Height == LOW){
        return PIDElevator(ELEV_LOW);
    }
    else if (Height == TRAP){
        return PIDElevator(ELEV_TRAP);
    }
    return false;
}
