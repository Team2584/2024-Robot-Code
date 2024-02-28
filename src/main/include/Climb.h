#include "Robot.h"
#include "constants/ClimbConstants.h"
#include "VisionBasedSwerve.h" 

#ifndef CLIMB_H
#define CLIMB_H

class Climb {

    private:

       
        VisionSwerve* robotSwerveDrive;
        frc::TrapezoidProfile<units::meters>::Constraints m_linearconstraints;
        frc::TrapezoidProfile<units::radians>::Constraints m_rotationconstraints;
        frc::ProfiledPIDController<units::meters> leftPID, rightPID;
        frc::ProfiledPIDController<units::radians> rollPID;

    public:

        bool climbZeroed = false;
        rev::CANSparkFlex leftClimbMotor, rightClimbMotor;
        rev::SparkRelativeEncoder leftEncoder, rightEncoder;
        rev::SparkLimitSwitch leftHallSensor, rightHallSensor;

        Climb(VisionSwerve* _swerveDrive);

        void SetClimbZero();

        bool GetLStop();

        bool GetRStop();

        bool ZeroClimb();

        void SetClimbMotors(double Percentage);

        void SetClimbMotors(double LeftMotor, double RightMotor);

        void ExtendClimb();

        void RetractClimb();

        void HoldClimb();

        bool ClimbPID(units::meter_t setpoint);

        bool BalanceAtPos();

        bool BalanceWhileClimbing();

        bool BalanceWhileClimbing(units::meter_t setpoint);

        bool GetClimbAtPos();
        
        bool GetClimbBalanced();

        bool GetClimbDone();

};

#endif