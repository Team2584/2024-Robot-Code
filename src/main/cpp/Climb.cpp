#include "AprilTagBasedSwerve.h"
#include "Climb.h"

Climb::Climb(AprilTagSwerve* _swerveDrive)
    :leftClimbMotor{CLIMB_MOTOR_L, rev::CANSparkMax::MotorType::kBrushless},
    rightClimbMotor(CLIMB_MOTOR_R, rev::CANSparkMax::MotorType::kBrushless),
    leftEncoder{leftClimbMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
    rightEncoder{rightClimbMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor)},
    robotSwerveDrive{_swerveDrive}
{
    leftClimbMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    rightClimbMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Climb:: ExtendClimb(){
    leftClimbMotor.Set(CLIMB_PCT_UP);
    rightClimbMotor.Set(CLIMB_PCT_UP);
}

void Climb:: RetractClimb(){
    leftClimbMotor.Set(CLIMB_PCT_DOWN*-1);
    rightClimbMotor.Set(CLIMB_PCT_DOWN*-1);
}

void Climb::HoldClimb(){
    leftClimbMotor.Disable();
    rightClimbMotor.Disable();
}

bool Climb::LiftRobotAuto(){

}

bool Climb::LowerRobotAuto(){

}

bool Climb::BalanceAtPos(){

}
