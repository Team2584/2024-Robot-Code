#include "VisionBasedSwerve.h"
#include "Climb.h"

Climb::Climb(VisionSwerve* _swerveDrive)
    :leftClimbMotor{CLIMB_MOTOR_L, rev::CANSparkFlex::MotorType::kBrushless},
    rightClimbMotor(CLIMB_MOTOR_R, rev::CANSparkFlex::MotorType::kBrushless),
    leftEncoder{leftClimbMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
    rightEncoder{rightClimbMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
    robotSwerveDrive{_swerveDrive},
    leftStop{CLIMB_LEFT_STOP_PORT},
    rightStop{CLIMB_RIGHT_STOP_PORT},
    leftPID{c_KP,c_KI,c_KD,c_KIMAX,c_MIN_SPEED,c_MAX_SPEED,c_POS_ERROR,c_VELOCITY_ERROR},
    rightPID{c_KP,c_KI,c_KD,c_KIMAX,c_MIN_SPEED,c_MAX_SPEED,c_POS_ERROR,c_VELOCITY_ERROR},
    rollPID{c_t_KP,c_t_KI,c_t_KD,c_t_KIMAX,c_t_MIN_SPEED,c_t_MAX_SPEED,c_t_ROT_ERROR,c_t_VELOCITY_ERROR}
{
    leftClimbMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    rightClimbMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

/**
 * @brief Brings the two climb motors down until they hit thier individual sensors. 
 * @note This MUST be called AND fully finished before non-roll Climb PID functions can be used 
*/
bool Climb::ZeroClimb(){
    if(!leftStop.Get() && !rightStop.Get() && !climbZeroed){
        if(!leftStop.Get()){
            leftClimbMotor.Set(CLIMB_PCT_DOWN*-1);
        }
        else{
            leftClimbMotor.Disable();
        }
        if(!rightStop.Get()){
            rightClimbMotor.Set(CLIMB_PCT_DOWN*-1);
        }
        else{
            rightClimbMotor.Disable();
        }
        return false;
    }
    else if (!climbZeroed){
        leftEncoder.SetPosition(0);
        rightEncoder.SetPosition(0);
        climbZeroed = true;
    }
    return true;
}

/**
 * @brief Move Both Climb Arms Up
*/
void Climb::ExtendClimb(){
    leftClimbMotor.Set(CLIMB_PCT_UP);
    rightClimbMotor.Set(CLIMB_PCT_UP);
}

/**
 * @brief Move Both Climb Arms Down
*/
void Climb::RetractClimb(){
    leftClimbMotor.Set(CLIMB_PCT_DOWN*-1);
    rightClimbMotor.Set(CLIMB_PCT_DOWN*-1);
}

/**
 * @brief Stop Climb in Place (Disables motor until next .Set)
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
bool Climb::ClimbPID(double setpoint){
    if(!ZeroClimb()){
        leftClimbMotor.Set(leftPID.Calculate(leftEncoder.GetPosition(), setpoint));
        rightClimbMotor.Set(rightPID.Calculate(rightEncoder.GetPosition(), setpoint));
        return (leftPID.PIDFinished() && rightPID.PIDFinished());
    }
    return false;
}

/** 
 * Use this function for tuning the climb PID
 * @brief PID Balance at the current point. Moves both arms equal and opposite distances.
 * @return True if the robot is ~horizontal
*/
bool Climb::BalanceAtPos(){
    double rotation = robotSwerveDrive->GetIMURoll();
    double error = rotation < 180 ? rotation : 360 - rotation;

    leftClimbMotor.Set(rollPID.Calculate(error,0)*-1);
    rightClimbMotor.Set(rollPID.Calculate(error,0));

    return rollPID.PIDFinished();
}

/**
 * @brief Climb at a defined speed while keeping the robot level
 * @return True if the robot is ~horizontal
*/
bool Climb::BalanceWhileClimbing(){
    double rotation = robotSwerveDrive->GetIMURoll();
    double error = rotation < 180 ? rotation : 360 - rotation;

    leftClimbMotor.Set(rollPID.Calculate(error,0)*-1 + CLIMB_PCT_DOWN*-1);
    rightClimbMotor.Set(rollPID.Calculate(error,0) + CLIMB_PCT_DOWN*-1);

    return rollPID.PIDFinished();
}


//PID climb to point while maintining level. 
//The logic is probably correct, not 100% sure
/**
 * @brief PID Climb to a set point while keeping the robot level
 * @param setpoint The setpoint in motor rotations
*/
bool Climb::BalanceWhileClimbing(double setpoint){
    if(!ZeroClimb()){
        double rotation = robotSwerveDrive->GetIMURoll();
        double error = rotation < 180 ? rotation : 360 - rotation;

        double leftPIDOutput = 0;
        double rightPIDOutput = 0;

        if(!GetClimbAtPos()){
            leftPIDOutput = leftPID.Calculate(leftEncoder.GetPosition(), setpoint);
            rightPIDOutput = rightPID.Calculate(rightEncoder.GetPosition(), setpoint);
        }

        double rollPIDOutput = rollPID.Calculate(error,0);

        leftClimbMotor.Set(leftPIDOutput - rollPIDOutput);
        rightClimbMotor.Set(rightPIDOutput + rollPIDOutput);

        return GetClimbAtPos();
    }
    return false;
}

/**
 * @brief Check if the Climb PID is Finished
 * @return True if the robot's most extended arm is at setpoint 
*/
bool Climb::GetClimbAtPos(){
    
    double lowPos = leftEncoder.GetPosition() < rightEncoder.GetPosition() ? rightEncoder.GetPosition() : leftEncoder.GetPosition();
    bool isAtPos =  abs(lowPos - rightPID.GetPIDSetpoint()) < rightPID.GetPIDAllowedError(); //if most extended arm ~= the set pos
    return(isAtPos); 
    
}

/**
 * @return If the robot is balanced
*/
bool Climb::GetClimbBalanced(){
    return rollPID.PIDFinished();
}

/**
 * @return If the robot is balanced and the robot is climbed enough (probably not very useful)
*/
bool Climb::GetClimbDone(){
    return (GetClimbBalanced() && GetClimbAtPos());
}

