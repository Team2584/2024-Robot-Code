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
        double GetSpinEncoderRadians(); // TODO make this function work
        SwerveModulePosition GetSwerveModulePosition();
        void ResetEncoders(); 
        void StopSwerveModule();
        void DriveSwerveModulePercent(double driveSpeed, double targetAngle);
};


/**
 * This class has functions to control all the drive control functionality of a swerve drive
 */
class SwerveDrive
{
    private:
        ctre::phoenix6::hardware::Pigeon2 *pigeonIMU;
        Translation2d FLWheelPos, FRWheelPos, BLWheelPos, BRWheelPos; /* Location of each wheel in relation to the center of the robot */ 
        wpi::array<Translation2d, 4> wheelPositionsArray; /* Array of all wheel positions */
        SwerveDriveKinematics<4> kinematics; /* A WPI struct which contains all wheels of a swerve drive*/
        SwerveDriveOdometry<4> *odometry; /* An odometry class which returns the position of the robot using wheel encoder ticks*/

   public:
        SwerveModule *FLModule, *FRModule, *BRModule, *BLModule; /* The four swerve modules at each corner of the robot */

        SwerveDrive(ctre::phoenix6::hardware::TalonFX *FLDriveMotor,
            rev::CANSparkMax *FLSpinMotor, frc::DutyCycleEncoder *FLMagEncoder,
            ctre::phoenix6::hardware::TalonFX *FRDriveMotor,
            rev::CANSparkMax *FRSpinMotor, frc::DutyCycleEncoder *FRMagEncoder,
            ctre::phoenix6::hardware::TalonFX *BRDriveMotor,
            rev::CANSparkMax *BRSpinMotor, frc::DutyCycleEncoder *BRMagEncoder,
            ctre::phoenix6::hardware::TalonFX *BLDriveMotor,
            rev::CANSparkMax *BLSpinMotor, frc::DutyCycleEncoder *BLMagEncoder,
            ctre::phoenix6::hardware::Pigeon2 *_pigeonIMU);

        double GetIMUHeading();
        wpi::array<SwerveModulePosition, 4> GetSwerveModulePositions();
        void ResetOdometry();
        void ResetOdometry(Pose2d position);
        Pose2d GetOdometryPose();
        void Update();
        double VelocityToPercent(double velocity);
        double PercentToVelocity(double percent);
        double AngularVelocityToPercent(double velocity);
        double AngularPercentToVelocity(double percent);
        void DriveSwervePercentNonFieldOriented(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed);
        void DriveSwervePercent(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed);
        void DriveSwerveMetersAndRadians(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed);
};