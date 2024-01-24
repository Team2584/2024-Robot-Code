#include "Robot.h"
#include "constants/ClimbConstants.h"

class Climb {

    public:
    
    rev::CANSparkMax leftClimbMotor, rightClimbMotor;
    rev::SparkRelativeEncoder leftEncoder, rightEncoder;
    AprilTagSwerve* robotSwerveDrive;

    Climb(AprilTagSwerve* _swerveDrive);

    void ExtendClimb();
    void RetractClimb();
    void HoldClimb();
    bool LiftRobotAuto();
    bool LowerRobotAuto();
    bool BalanceAtPos();


};