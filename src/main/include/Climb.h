#include "Robot.h"
#include "constants/ClimbConstants.h"

class Climb {

    private:

        rev::CANSparkFlex leftClimbMotor, rightClimbMotor;
        rev::SparkRelativeEncoder leftEncoder, rightEncoder;
        VisionSwerve* robotSwerveDrive;
        rev::SparkLimitSwitch leftStop;
        rev::SparkLimitSwitch rightStop;
        PID leftPID;
        PID rightPID;
        PID rollPID;

        

    public:
    bool climbZeroed = false;
        rev::SparkRelativeEncoder leftEncoder, rightEncoder;
        frc::DigitalInput leftStop;
        frc::DigitalInput rightStop;

        Climb(VisionSwerve* _swerveDrive);

        bool GetLStop();

        bool GetRStop();

        bool ZeroClimb();

        void SetClimbMotors(double Percentage);

        void SetClimbMotors(double LeftMotor, double RightMotor);

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