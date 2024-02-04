#include "AprilTagBasedSwerve.h"
#include "Climb.h"


Climb::Climb(AprilTagSwerve* _swerveDrive)
    :leftClimbMotor{CLIMB_MOTOR_L, rev::CANSparkMax::MotorType::kBrushless},
    rightClimbMotor(CLIMB_MOTOR_R, rev::CANSparkMax::MotorType::kBrushless),
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

//Bad implementation, fix later (call at begninning of teleopinit for testing once motor direction is found), 
//forces motors to move until climb zero is hit
void Climb::ZeroClimb(){
    while(!leftStop.Get() && !rightStop.Get()){
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
    }
    leftEncoder.SetPosition(0);
    rightEncoder.SetPosition(0);
    climbZeroed = true;
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
 * Disables so there aren't any issues end of match
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
    if(climbZeroed){
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
///THIS FUNCTION IS NOT COMPLETE!!!!
///The logic for the arm PIDs isn't correct, as they won't nessesarily be aiming for the same point, 
//just whatever point makes the conditions in GetClimbAtPos() true
//this is a problem for later once everything else is tested :)
//maybe just make 1 arm be the min value?
/**
 * @brief PID Climb to a set point while keeping the robot level
 * @param setpoint The setpoint in motor rotations
*/
bool Climb::BalanceWhileClimbing(double setpoint){
    if(climbZeroed){
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
 * @return True if the robot is ~horizontal and the most extended arm is at setpoint
*/
bool Climb::GetClimbAtPos(){
    
    double lowPos = leftEncoder.GetPosition() < rightEncoder.GetPosition() ? rightEncoder.GetPosition() : leftEncoder.GetPosition();
    bool isAtPos =  abs(lowPos - rightPID.GetPIDSetpoint()) < rightPID.GetPIDAllowedError(); //if most extended arm ~= the set pos
    return(isAtPos); 
    
    /*
    //this code is similar but checks if the avg position of the arms is at the setpoint. may or may not be useful
    double avgPos = (leftEncoder.GetPosition() + rightEncoder.GetPosition())/2.0; 
    bool isAtPos =  abs(avgPos - rightPID.GetPIDSetpoint()) < rightPID.GetPIDAllowedError(); //if the arm's average pos ~= the set pos
    */
}

bool Climb::GetClimbBalanced(){
    return rollPID.PIDFinished();
}

bool Climb::GetClimbDone(){
    return (GetClimbBalanced() && GetClimbAtPos());
}

