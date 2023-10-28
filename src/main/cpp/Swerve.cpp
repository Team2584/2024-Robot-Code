#include "Swerve.h"

/**
 * Constructor for a Swerve Module: One of the four 2-motor systems in a swerve drive.
 *
 * @param driveMotor_ A pointer to a Talon FX Drive Motor (the one that makes the robot move)
 * @param spinMotor_ A pointer to a Talon FX Spin Motor (the one that makes the wheel rotate to change direction)
 * @param magEncoder_ A pointer to the absolute encoder used to track rotation of the swerve module wheels
 */
SwerveModule::SwerveModule(ctre::phoenix6::hardware::TalonFX *driveMotor_,
                           rev::CANSparkMax *spinMotor_, frc::DutyCycleEncoder *magEncoder_,
                           double encoderOffset_)
    :spinPIDController{WHEEL_SPIN_KP, WHEEL_SPIN_KI, WHEEL_SPIN_KD}
{
    driveMotor = driveMotor_;
    spinMotor = spinMotor_;
    magEncoder = magEncoder_;
    encoderOffset = encoderOffset_;
    spinRelativeEncoder = new rev::SparkMaxRelativeEncoder(spinMotor->GetEncoder());

    spinPIDController.SetTolerance(-1 * WHEEL_SPIN_ERROR, WHEEL_SPIN_ERROR); 

    ResetEncoders();
}

/**
 * Finds the absolute heading of the swerve drive wheel relative to the robot.
 *
 * @return The Swerve Drive wheel's heading in degrees with 0.0 being the front of the robot increasing clockwise.
 */
double SwerveModule::GetModuleHeading()
{
    double encoderReading = magEncoder->GetAbsolutePosition();
    // subtract the encoder offset to make 0 degrees forward
    encoderReading -= encoderOffset;
    if (encoderReading < 0)
        encoderReading += 1;
    // Flip the degrees to make clockwise positive
    encoderReading = 1 - encoderReading;
    // Convert from 0-1 to degrees
    encoderReading *= 360;
    return encoderReading;
}

/**
 *  Returns Talon Drive Encoder
 */
double SwerveModule::GetDriveEncoder()
{
    // acquire a refreshed TalonFX rotor position signal
    auto &rotorPosSignal = driveMotor->GetRotorPosition();
    return rotorPosSignal.GetValue().value();
}

/**
 *  Converts Talon Drive Encoder Reading to Meters
 */
double SwerveModule::GetDriveEncoderMeters()
{
    return (GetDriveEncoder() - driveEncoderInitial) / 2048 / DRIVE_MOTOR_GEAR_RATIO * DRIVE_MOTOR_CIRCUMFERENCE;
}

/**
 *  Converts Talon Drive Encoder Reading to Meters per Second
 */
double SwerveModule::GetDriveVelocity()
{
    auto &rotorPosSignal = driveMotor->GetVelocity();
    return rotorPosSignal.GetValue().value() / 2048 / 6.54 * 0.10322 * M_PI * 10;
}

/**
 *  INCOMPLETE DO NOT USE UNDER ANY CIRCUMSTANCE, USE GetModuleHeading() INSTEAD!
 *  Also this hasn't been updated from the talon swerve drive as it is an uneccessary functino
 *  This function was intended to use the relative encoder as a backup if the absolute mag encoder broke
 */
double SwerveModule::GetSpinEncoderRadians()
{
    return spinRelativeEncoder->GetPosition();
}

/**
 * Resets the drive and spin encoders to 0, usually unessecesary as odometry works through change in encoder readings.
 */
void SwerveModule::ResetEncoders()
{
    driveEncoderInitial = GetDriveEncoder();
    spinEncoderInitialHeading = GetModuleHeading();
}

/**
 *  Setss all motor speeds to 0.
 */
void SwerveModule::StopSwerveModule()
{
    spinMotor->Set(0);
    driveMotor->Set(0);
}

/**
 * Drives the Swerve Module at a certaint speed in a certain direction using a simple P feedback loop.
 * Utitlizes half-baked logic to decide the most efficient way for the swerve drive to spin the wheel.
 * Ask Avrick for a more detailed explanation of how this function works, it came to him in a dream.
 *
 * @param driveSpeed The desired velocity of the wheel in percent of maximum (0 - 1.0)
 * @param targetAngle The desired angle of the wheel in Degrees
 */
void SwerveModule::DriveSwerveModulePercent(double driveSpeed, double targetAngle)
{
    // current encoder reading
    double wheelAngle = GetModuleHeading();
    // amount wheel has left to turn to reach target
    double error = 0;
    // if wheel should spin clockwise(1) or counterclockwise(-1) to reach the target
    int spinDirection = 0;
    // If the drive should spin forward(1) or backward(-1) to move in the correct direction
    int driveDirection = 0;

    // Corrects spin angle to make it positive
    if (targetAngle < 0)
    {
        targetAngle += 360;
    }

    // The below logic determines the most efficient way for the wheel to move to reach the desired angle
    // This could mean moving towards it clockwise, counterclockwise, or moving towards the opposite of the angle
    // and driving in the opposite direction
    if (wheelAngle < targetAngle)
    {
        // if target and wheelangle are less than 90 degrees apart we should spin directly towards the target angle
        if (targetAngle - wheelAngle <= 90)
        {
            error = targetAngle - wheelAngle;
            spinDirection = 1;
            driveDirection = 1;
        }
        // else if target angle is "1 quadrant" away from the wheel Angle spin counterclockwise to the opposite of
        // targetAngle and spin the drive motors in the opposite direction
        else if (targetAngle - wheelAngle <= 180)
        {
            // Distance the wheel must spin is now not the distance between the target and the wheelAngle, but rather
            // the distance between the opposite of the target and the wheelAngle
            error = 180 - (targetAngle - wheelAngle);
            spinDirection = -1;
            driveDirection = -1;
        }
        else if (targetAngle - wheelAngle <= 270)
        {
            error = (targetAngle - wheelAngle) - 180;
            spinDirection = 1;
            driveDirection = -1;
        }
        // if target and wheelAngle are less than 90 degrees apart we should spin directly towards the target angle
        // Here however, we must reverse the spin direction because the target is counterclockwise of the wheelAngle
        else
        {
            error = 360 - (targetAngle - wheelAngle);
            spinDirection = -1;
            driveDirection = 1;
        }
    }
    else if (wheelAngle > targetAngle)
    {
        // The logic below is similar to the logic above, but in the case where wheelAngle > targetAngle
        if (wheelAngle - targetAngle <= 90)
        {
            error = wheelAngle - targetAngle;
            spinDirection = -1;
            driveDirection = 1;
        }
        else if (wheelAngle - targetAngle <= 180)
        {
            error = 180 - (wheelAngle - targetAngle);
            spinDirection = 1;
            driveDirection = -1;
        }
        else if (wheelAngle - targetAngle <= 270)
        {
            error = (wheelAngle - targetAngle) - 180;
            spinDirection = -1;
            driveDirection = -1;
        }
        else
        {
            error = 360 - (wheelAngle - targetAngle);
            spinDirection = 1;
            driveDirection = 1;
        }
    }

    // PID to spin the wheel, makes the wheel move slower as it reaches the target 
    // (error divided by 90 because that is the furthest the wheel will possibly have to move)
    double spinMotorSpeed = spinPIDController.Calculate(0, error / 90.0);
    spinMotorSpeed = std::clamp(spinMotorSpeed, -1 * WHEEL_SPIN_MAX_SPEED, WHEEL_SPIN_MAX_SPEED);

    // Move motors  at speeds and directions determined earlier
    spinMotor->Set(spinMotorSpeed * spinDirection);
    driveMotor->Set(driveSpeed * driveDirection);
}