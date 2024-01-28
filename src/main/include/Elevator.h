#include "Robot.h"
#include "constants/ElevatorConstants.h"

#ifndef ELEVATOR_H
#define ELEVATOR_H

class Elevator
{
    public:

    double toflastSpeed = 0;
    double toflastHeight = 0;

    rev::CANSparkMax winchMotor;
    rev::SparkRelativeEncoder *winchEncoder;
    rev::CANSparkMax ampMotor;
    frc::DigitalInput ampMechSensor;

    frc::TrapezoidProfile<units::meters>::Constraints m_constraints{e_kMaxVelocity, e_kMaxAcceleration};
    frc::ProfiledPIDController<units::meters> m_controller{e_kP, e_kI, e_kD, m_constraints};
    frc::ElevatorFeedforward m_feedforward{e_kS, e_kG, e_kV};

    Elevator();

    void ResetElevatorEncoder();

    void SetAmpMotor(double speed);

    double GetWinchEncoderReading();

    double TOFSReading();

    double TOFSElevatorHeight();

    void StopElevator();

    void MoveElevatorPercent(double percent);

    bool PIDElevator(double setpoint);

    bool GetObjectInMech();

    bool GetElevatorAtSetpoint();


};

#endif