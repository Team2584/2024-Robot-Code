#include "Robot.h"
#include "constants/ClimbConstants.h"

class Climb {

    public:

    bool climbZeroed = false;

    rev::CANSparkMax leftClimbMotor, rightClimbMotor;
    rev::SparkRelativeEncoder leftEncoder, rightEncoder;
    AprilTagSwerve* robotSwerveDrive;
    frc::DigitalInput leftStop;
    frc::DigitalInput rightStop;
    PID leftPID;
    PID rightPID;
    PID rollPID;

    Climb(AprilTagSwerve* _swerveDrive);

    void ZeroClimb();
    void ExtendClimb();
    void RetractClimb();
    void HoldClimb();
    bool LiftRobotAuto();
    bool LowerRobotAuto();
    bool ClimbPID(double setpoint);
    bool BalanceAtPos();
    bool BalanceWhileClimbing();
    bool BalanceWhileClimbing(double setpoint);
    bool GetClimbAtPos();

};