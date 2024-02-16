#include "Robot.h"
#include "constants/ClimbConstants.h"

class Climb {

    private:

        rev::CANSparkFlex leftClimbMotor, rightClimbMotor;
        rev::SparkRelativeEncoder leftEncoder, rightEncoder;
        AprilTagSwerve* robotSwerveDrive;
        rev::SparkLimitSwitch leftStop;
        rev::SparkLimitSwitch rightStop;
        PID leftPID;
        PID rightPID;
        PID rollPID;

        bool climbZeroed = false;

    public:

        Climb(AprilTagSwerve* _swerveDrive);

        void SetClimbMotors(double left, double right);

        bool ZeroClimb();

        void ExtendClimb();

        void RetractClimb();

        void HoldClimb();

        bool ClimbPID(double setpoint);

        bool BalanceAtPos();

        bool BalanceWhileClimbing();

        bool BalanceWhileClimbing(double setpoint);

        bool GetClimbAtPos();
        
        bool GetClimbBalanced();

        bool GetClimbDone();

};