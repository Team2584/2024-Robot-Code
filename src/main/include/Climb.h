#include "Robot.h"
#include "constants/ClimbConstants.h"

class Climb {

    private:

        rev::CANSparkFlex leftClimbMotor, rightClimbMotor;
        rev::SparkRelativeEncoder leftEncoder, rightEncoder;
        VisionSwerve* robotSwerveDrive;
        frc::DigitalInput leftStop;
        frc::DigitalInput rightStop;
        PID leftPID;
        PID rightPID;
        PID rollPID;

        bool climbZeroed = false;

    public:

        Climb(VisionSwerve* _swerveDrive);

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