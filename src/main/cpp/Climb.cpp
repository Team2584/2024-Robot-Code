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
//forces motors to move until climb zerois hit
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

//Move both arms up
void Climb::ExtendClimb(){
    leftClimbMotor.Set(CLIMB_PCT_UP);
    rightClimbMotor.Set(CLIMB_PCT_UP);
}

//move both arms down
void Climb::RetractClimb(){
    leftClimbMotor.Set(CLIMB_PCT_DOWN*-1);
    rightClimbMotor.Set(CLIMB_PCT_DOWN*-1);
}

//Stop climb in place
void Climb::HoldClimb(){
    leftClimbMotor.Disable();
    rightClimbMotor.Disable();
}

//*****these two functions should be used for tuning the PID, not very useful after as BalanceWhileClimbing(double setpoint does it at the same time)

//pid both arms to a point
bool Climb::ClimbPID(double setpoint){
    if(climbZeroed){
        leftClimbMotor.Set(leftPID.Calculate(leftEncoder.GetPosition(), setpoint));
        rightClimbMotor.Set(rightPID.Calculate(rightEncoder.GetPosition(), setpoint));
        return (leftPID.PIDFinished() && rightPID.PIDFinished());
    }
    return false;
}

//balance at the current point
bool Climb::BalanceAtPos(){
    double rotation = robotSwerveDrive->GetIMURoll();
    double rollVelocity = robotSwerveDrive->GetRollSpeed();
    double error = rotation - 360;

    leftClimbMotor.Set(rollPID.Calculate(error,0)*-1);
    rightClimbMotor.Set(rollPID.Calculate(error,0));

    return rollPID.PIDFinished();
}

//****Actual PID climb functions

//simple Climb while maintaining level
bool Climb::BalanceWhileClimbing(){
    double rotation = robotSwerveDrive->GetIMURoll();
    double error = rotation < 180 ? rotation : 360 - rotation;

    leftClimbMotor.Set(rollPID.Calculate(error,0)*-1 + CLIMB_PCT_UP);
    rightClimbMotor.Set(rollPID.Calculate(error,0) + CLIMB_PCT_UP);

    return GetClimbAtPos();
}

//PID climb to point while maintining level. 

///THIS FUNCTION IS NOT COMPLETE!!!!
///The logic for the arm PIDs isn't correct, as they won't nessesarily be aiming for the same point, 
//just whatever point makes the conditions in GetClimbAtPos() true
//this is a problem for later once everything else is tested :)

bool Climb::BalanceWhileClimbing(double setpoint){
    if(climbZeroed){
        double rotation = robotSwerveDrive->GetIMURoll();
        double error = rotation < 180 ? rotation : 360 - rotation;

        double leftPIDOutput = leftPID.Calculate(leftEncoder.GetPosition(), setpoint);
        double rightPIDOutput = rightPID.Calculate(rightEncoder.GetPosition(), setpoint);
        double rollPIDOutput = rollPID.Calculate(error,0);

        leftClimbMotor.Set(leftPIDOutput - rollPIDOutput);
        rightClimbMotor.Set(rightPIDOutput + rollPIDOutput);

        return GetClimbAtPos();
    }
    return false;
}

bool Climb::GetClimbAtPos(){
    double avgPos = (leftEncoder.GetPosition() + rightEncoder.GetPosition())/2.0; 
    bool isAtPos =  abs(avgPos - rightPID.GetPIDSetpoint()) < rightPID.GetPIDAllowedError(); //if the arm's average pos ~= the set pos
    bool isBalanced = rollPID.PIDFinished(); //and robot is ~=horizontal
    return(isAtPos && isBalanced); 
}


