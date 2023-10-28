#include "Robot.h"
#include "SwerveConstants.h"

/**
 * A Swerve Drive has 4 Swerve Modules, each of which is defined as an object of the class SwerveModule
 * In general, this class has functions allowing us to control each swerve module individually.
 */
class SwerveModule
{
    private:
        // Instance Variables for each swerve module
        ctre::phoenix6::hardware::TalonFX *driveMotor; /* The motor responsible for actually driving the wheel*/
        rev::CANSparkMax *spinMotor; /* The motor responsible for "spinning" the wheel left to right to change direction*/
        rev::SparkMaxRelativeEncoder *spinRelativeEncoder; /* The relative encoder built into the spinMotor */
        PIDController spinPIDController; /* The PID Controller for the spinMotor, works using degrees */
        double encoderOffset;       /* Offset in magnetic encoder from 0 facing the front of the robot */
        double driveEncoderInitial; /* Used to computer the change in encoder tics, aka motor rotation */
        double spinEncoderInitialHeading; /* Initial Heading of Relative Spin Encoder used for zeroing*/
        double spinEncoderInitialValue; /* Initial Value of Relative Spin Encoder used for zeroing*/

    public:
        frc::DutyCycleEncoder *magEncoder;

        SwerveModule(ctre::phoenix6::hardware::TalonFX *driveMotor_,
                    rev::CANSparkMax *spinMotor_, frc::DutyCycleEncoder *magEncoder_,
                    double encoderOffset_);

        double GetModuleHeading();
        double GetDriveEncoder();
        double GetDriveEncoderMeters();
        double GetDriveVelocity();
        double GetSpinEncoderRadians(); // curently unused
        void ResetEncoders(); 
        void StopSwerveModule();
        void DriveSwerveModulePercent(double driveSpeed, double targetAngle);
};