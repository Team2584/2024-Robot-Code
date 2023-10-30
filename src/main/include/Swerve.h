#include "Robot.h"
#include "Constants/SwerveConstants.h"

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
        PID spinPIDController; /* The PID Controller for the spinMotor, works using degrees */
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
        double GetSpinEncoderRadians(); // TODO make this function work
        void ResetEncoders(); 
        void StopSwerveModule();
        void DriveSwerveModulePercent(double driveSpeed, double targetAngle);
};


/**
 * This class has functions to control all the autonomous and drive control functionality of a swerve drive
 */
class SwerveDrive
{
    private:
        ctre::phoenix6::hardware::Pigeon2 *pigeonIMU;
        Translation2d frontLeftWheelPos, frontRightWheelPos, backLeftWheelPos, backRightWheelPos; /* Location of each wheel in relation to the center of the robot */    // TODO Make sure these are used

   public:
        SwerveModule *FLModule, *FRModule, *BRModule, *BLModule;
        double pigeonInitial = 0; /* the inital rotation value of the Pigeon IMU */    // TODO delete in future once we can rely on odometry class

        SwerveDrive(ctre::phoenix6::hardware::TalonFX *_FLDriveMotor,
            rev::CANSparkMax *_FLSpinMotor, frc::DutyCycleEncoder *_FLMagEncoder,
            double _FLEncoderOffset, ctre::phoenix6::hardware::TalonFX *_FRDriveMotor,
            rev::CANSparkMax *_FRSpinMotor, frc::DutyCycleEncoder *_FRMagEncoder,
            double _FREncoderOffset, ctre::phoenix6::hardware::TalonFX *_BRDriveMotor,
            rev::CANSparkMax *_BRSpinMotor, frc::DutyCycleEncoder *_BRMagEncoder,
            double _BREncoderOffset, ctre::phoenix6::hardware::TalonFX *_BLDriveMotor,
            rev::CANSparkMax *_BLSpinMotor, frc::DutyCycleEncoder *_BLMagEncoder,
            double _BLEncoderOffset, ctre::phoenix6::hardware::Pigeon2 *_pigeonIMU, 
            double initialHeading);

        double VelocityToPercent(double velocity);
        double PercentToVelocity(double percent);
        double AngularVelocityToPercent(double velocity);
        double AngularPercentToVelocity(double percent);
        double GetIMUHeading();
        void ResetIMU(); //TODO Delete
        void DriveSwervePercentNonFieldOriented(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed);
        void DriveSwervePercent(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed);
        void DriveSwerveMetersAndRadians(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed);
};