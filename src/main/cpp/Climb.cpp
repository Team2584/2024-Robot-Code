#include "VisionBasedSwerve.h"
#include "Climb.h"

Climb::Climb(VisionSwerve* _swerveDrive)
    :leftClimbMotor{CLIMB_MOTOR_L, rev::CANSparkFlex::MotorType::kBrushless},
    rightClimbMotor(CLIMB_MOTOR_R, rev::CANSparkFlex::MotorType::kBrushless),
    robotSwerveDrive{_swerveDrive},
    m_linearconstraints{ClimbConstants::Linear::kMaxVelocity,ClimbConstants::Linear::kMaxAcceleration},
    m_rotationconstraints{ClimbConstants::Rotation::kMaxVelocity,ClimbConstants::Rotation::kMaxAcceleration},
    leftPID{ClimbConstants::Linear::m_KP,ClimbConstants::Linear::m_KI,ClimbConstants::Linear::m_KD, m_linearconstraints},
    rightPID{ClimbConstants::Linear::m_KP,ClimbConstants::Linear::m_KI,ClimbConstants::Linear::m_KD, m_linearconstraints},
    rollPID{ClimbConstants::Rotation::m_KP,ClimbConstants::Rotation::m_KI,ClimbConstants::Rotation::m_KD, m_rotationconstraints},
    leftEncoder{leftClimbMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
    rightEncoder{rightClimbMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
    leftHallSensor{leftClimbMotor.GetForwardLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen)},
    rightHallSensor{rightClimbMotor.GetForwardLimitSwitch(rev::SparkLimitSwitch::Type::kNormallyOpen)}
{
    leftClimbMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    rightClimbMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    leftEncoder.SetPositionConversionFactor(ClimbConstants::CLIMB_CONVERSION_FACTOR);
    rightEncoder.SetPositionConversionFactor(ClimbConstants::CLIMB_CONVERSION_FACTOR);
    leftPID.SetTolerance(ClimbConstants::Linear::m_POS_ERROR);
    rightPID.SetTolerance(ClimbConstants::Linear::m_POS_ERROR);
    leftClimbMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, false);
    leftClimbMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, false);          
    rightClimbMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, false);
    rightClimbMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, false);
    leftEncoder.SetPosition(100);
    rightEncoder.SetPosition(-100);
}

bool Climb::GetLStop(){
    return leftHallSensor.Get();
}

bool Climb::GetRStop(){
    return rightHallSensor.Get();
}

void Climb::SetClimbZero(){
    
    leftEncoder.SetPosition(0);
    rightEncoder.SetPosition(0);
    rightClimbMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, ClimbConstants::MinHeight.value());
    leftClimbMotor.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, ClimbConstants::MinHeight.value());

    leftClimbMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, false);
    leftClimbMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, true);          
    rightClimbMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, false);
    rightClimbMotor.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, true);
    
    //add soft limits
}

/**
 * @brief Brings the two climb motors down until they hit thier individual sensors. 
 * @note This MUST be called AND fully finished before non-roll Climb PID functions can be used 
*/
bool Climb::ZeroClimb(){
    SmartDashboard::PutBoolean("Climb Zeroed", climbZeroed);
    if(!climbZeroed){
        
        if(!GetLStop()){
            leftClimbMotor.Set(ClimbConstants::BasePctDown*-0.9);
        }
        else{
            leftClimbMotor.Disable();
        }
        if(!GetRStop()){
            rightClimbMotor.Set(ClimbConstants::BasePctDown*0.9);
        }
        else{
            rightClimbMotor.Disable();
        }
        if (GetRStop() && GetLStop()){
            SetClimbZero();
            climbZeroed = true;
            return true;
        }
    }
    else {
        return true;
    }
    return false;
}

/**
 * @brief Set climb motor percentage. 
 * @note A negative percentage brings the climb arms down, positive up
*/
void Climb::SetClimbMotors(double Percentage){
    SetClimbMotors(Percentage, Percentage);
}

/**
 * @brief Set climb motor percentage. 
 * @note A negative percentage brings the climb arms down, positive up
*/
void Climb::SetClimbMotorsBorked(double LeftMotor, double RightMotor){
    leftClimbMotor.Set(LeftMotor);
    rightClimbMotor.Set(RightMotor*-1);
}

/**
 * @brief Set climb motor percentage. 
 * @note A negative percentage brings the climb arms down, positive up
*/
void Climb::SetClimbMotors(double LeftMotor, double RightMotor){
    if (leftEncoder.GetPosition() < -0.005 && LeftMotor < 0 && lClimbZeroed)
        leftClimbMotor.Set(0);
    else
        leftClimbMotor.Set(LeftMotor);

    if (rightEncoder.GetPosition() > 0.005 && RightMotor < 0 && rClimbZeroed)   
        rightClimbMotor.Set(0);
    else
        rightClimbMotor.Set(RightMotor*-1);

}

/**
 * @brief Move Both Climb Arms Up
*/
void Climb::ExtendClimb(){
    SetClimbMotors(ClimbConstants::BasePctUp);
}

/**
 * @brief Move Both Climb Arms Down
*/
void Climb::RetractClimb(){
    SetClimbMotors(ClimbConstants::BasePctDown*-1);
}

/**
 * @brief HallSensor Climb in Place (Disables motor until next .Set)
 * @note Disables so there aren't any issues end of match
*/
void Climb::HoldClimb(){
    leftClimbMotor.Disable();
    rightClimbMotor.Disable();
}

/** 
 * Use this function for tuning the climb PID
 * @brief PID both arms to a setpoint (seperately)
 * @param setpoint The setpoint in motor rotations 
 * @return True if both arms are at setpoint
*/
bool Climb::ClimbPID(units::meter_t setpoint){
    if(ZeroClimb()){
        leftPID.SetGoal(setpoint);
        rightPID.SetGoal(setpoint*-1);

                SmartDashboard::PutNumber("Climb setpoint", leftPID.GetGoal().position.value());

        units::volt_t left = units::volt_t{leftPID.Calculate(units::meter_t{leftEncoder.GetPosition()})};
        units::volt_t right = units::volt_t{rightPID.Calculate(units::meter_t{rightEncoder.GetPosition()})};

        SmartDashboard::PutNumber("Climb L Error",leftPID.GetPositionError().value());
        SmartDashboard::PutNumber("Climb L Pos",leftEncoder.GetPosition());
        SmartDashboard::PutNumber("Climb L Voltage Send", left.value());
        SmartDashboard::PutNumber("Climb L Voltage Recv", leftClimbMotor.GetBusVoltage());
        SmartDashboard::PutNumber("Climb L Current", leftClimbMotor.GetOutputCurrent());

        SmartDashboard::PutNumber("Climb r Error",rightPID.GetPositionError().value());
        SmartDashboard::PutNumber("Climb r Pos",rightEncoder.GetPosition());
        SmartDashboard::PutNumber("Climb r Voltage Send", right.value());
        SmartDashboard::PutNumber("Climb r Voltage Recv", rightClimbMotor.GetBusVoltage());
        SmartDashboard::PutNumber("Climb r Current", rightClimbMotor.GetOutputCurrent());

        leftClimbMotor.SetVoltage(left);
        rightClimbMotor.SetVoltage(right);

        return (leftPID.AtGoal() && rightPID.AtGoal());
    }
    return false;
}

/** 
 * @note Use this function for tuning the climb PID
 * @brief PID Balance at the current point. Moves both arms equal and opposite distances.
 * @return True if the robot is ~horizontal
*/
bool Climb::BalanceAtPos(){

    rollPID.SetGoal(0_rad);

    auto rotation = units::degree_t{robotSwerveDrive->GetIMURoll()};
    units::radian_t error = rotation < 180_deg ? rotation : 360_deg - rotation;

    double left = rollPID.Calculate(error)*-1;
    double right = rollPID.Calculate(error);

    SetClimbMotors(left,right);

    return rollPID.AtGoal();
}

/**
 * @brief Climb at a defined speed while keeping the robot level
 * @return True if the robot is ~horizontal
*/
bool Climb::BalanceWhileClimbing(){

    rollPID.SetGoal(0_rad);

    auto rotation = units::degree_t{robotSwerveDrive->GetIMURoll()};
    units::radian_t error = rotation < 180_deg ? rotation : 360_deg - rotation;

    double left = rollPID.Calculate(error)*-1 + ClimbConstants::BasePctDown*-1;
    double right = rollPID.Calculate(error) + ClimbConstants::BasePctDown*-1;

    SetClimbMotors(left,right);

    return rollPID.AtGoal();
}


//PID climb to point while maintining level. 
//The logic is probably correct, not 100% sure
/**
 * @brief PID Climb to a set point while keeping the robot level
 * @param setpoint The setpoint in motor rotations
*/
bool Climb::BalanceWhileClimbing(units::meter_t setpoint){
    if(ZeroClimb()){

        rollPID.SetGoal(0_rad);
        leftPID.SetGoal(setpoint);
        rightPID.SetGoal(setpoint);

        auto rotation = units::degree_t{robotSwerveDrive->GetIMURoll()};
        units::radian_t error = rotation < 180_deg ? rotation : 360_deg - rotation;

        units::volt_t leftPIDOutput = 0_V;
        units::volt_t rightPIDOutput = 0_V;

        if(!GetClimbAtPos()){
            leftPIDOutput = units::volt_t{leftPID.Calculate(units::meter_t{leftEncoder.GetPosition()})};
            rightPIDOutput = units::volt_t{rightPID.Calculate(units::meter_t{rightEncoder.GetPosition()*-1})*-1};
        }

        units::volt_t rollPIDOutput = units::volt_t{rollPID.Calculate(error)};
        
        units::volt_t left = leftPIDOutput - rollPIDOutput;
        units::volt_t right = rightPIDOutput + rollPIDOutput;

        leftClimbMotor.SetVoltage(left);
        rightClimbMotor.SetVoltage(right);

        return GetClimbAtPos();
    }
    return false;
}

/**
 * @brief Check if the Climb PID is Finished
 * @return True if both arms are at the setpoint
 * @note This should be later corrected for if the arms are at different heights but the robot is climbed enough
*/
bool Climb::GetClimbAtPos(){
    /*
    double lowPos = leftEncoder.GetPosition() < rightEncoder.GetPosition() ? rightEncoder.GetPosition() : leftEncoder.GetPosition();
    bool isAtPos =  abs(lowPos - rightPID.GetSetpoint()) < rightPID.GetPositionTolerance(); //if most extended arm ~= the set pos
    return(isAtPos); 
    */
   return leftPID.AtGoal() && rightPID.AtGoal();
    
}

/**
 * @return If the robot is balanced
*/
bool Climb::GetClimbBalanced(){
    return rollPID.AtGoal();
}

/**
 * @return If the robot is balanced and the robot is climbed enough
*/
bool Climb::GetClimbDone(){
    return (GetClimbBalanced() && GetClimbAtPos());
}

void Climb::UpdateClimbEncoders(){
    if (GetLStop())
    {
        leftEncoder.SetPosition(0);
        lClimbZeroed = true; 
    }
    if (GetRStop())
    {
        rightEncoder.SetPosition(0);
        rClimbZeroed = true; 
    }
}