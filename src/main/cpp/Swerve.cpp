#include "Swerve.h"

/*
  ____                               __  __           _       _
 / ___|_      _____ _ ____   _____  |  \/  | ___   __| |_   _| | ___
 \___ \ \ /\ / / _ \ '__\ \ / / _ \ | |\/| |/ _ \ / _` | | | | |/ _ \
  ___) \ V  V /  __/ |   \ V /  __/ | |  | | (_) | (_| | |_| | |  __/
 |____/ \_/\_/ \___|_|    \_/ \___| |_|  |_|\___/ \__,_|\__,_|_|\___|

*/

/**
 * Constructor for a Swerve Module: One of the four 2-motor systems in a swerve drive.
 *
 * @param driveMotor_ A pointer to a Talon FX Drive Motor (the one that makes the robot move)
 * @param spinMotor_ A pointer to a Talon FX Spin Motor (the one that makes the wheel rotate to change direction)
 * @param magEncoder_ A pointer to the absolute encoder used to track the rotation of the swerve module wheels
 */
SwerveModule::SwerveModule(int driveMotorPort, int spinMotorPort, int magneticEncoderPort,
                           double encoderOffset_)
    : driveMotor{driveMotorPort},
      spinMotor{spinMotorPort, rev::CANSparkMax::MotorType::kBrushless},
      spinRelativeEncoder{spinMotor.GetEncoder()},
      magEncoder{magneticEncoderPort},
      spinPIDController{WHEEL_SPIN_KP, WHEEL_SPIN_KI, WHEEL_SPIN_KD, WHEEL_SPIN_KI_MAX,
                        WHEEL_SPIN_MIN_SPEED, WHEEL_SPIN_MAX_SPEED,
                        WHEEL_SPIN_TOLERANCE, WHEEL_SPIN_VELOCITY_TOLERANCE}
{
    // Instantiates all variables needed for class
    encoderOffset = encoderOffset_;

    ResetEncoders();
}

/*
* Return the mag encoder's current value (NOT CORRECTED TO MAKE 0 DEGREES FORWARD)
*/
double SwerveModule::GetMagEncoderValue()
{
    return magEncoder.GetAbsolutePosition();
}

/**
 * Finds the absolute heading of the swerve drive wheel relative to the robot.
 *
 * @return The Swerve Drive wheel's heading in degrees, with 0.0 being the front of the robot increasing clockwise.
 */
double SwerveModule::GetModuleHeading()
{
    double encoderReading = GetMagEncoderValue();
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
 *  Returns Talon Drive Encoder Rotations
 */
double SwerveModule::GetDriveEncoder()
{
    // Acquire a refreshed TalonFX rotor position signal
    auto &rotorPosSignal = driveMotor.GetRotorPosition();
    return rotorPosSignal.GetValue().value();
}

/**
 *  Converts Talon Drive Encoder Reading to Meters
 */
double SwerveModule::GetDriveEncoderMeters()
{
    // 2048 is the resolution of the drive encoder, meaning the number added to the drive encoder for each rotation
    return (GetDriveEncoder() - driveEncoderInitial) / DRIVE_MOTOR_GEAR_RATIO * DRIVE_MOTOR_CIRCUMFERENCE;
}

/**
 *  INCOMPLETE DO NOT USE UNDER ANY CIRCUMSTANCE; USE GetModuleHeading() INSTEAD!
 *  Also, this hasn't been updated from the talon swerve drive, as it is an unnecessary function
 *  This function was intended to use the relative encoder as a backup if the absolute mag encoder broke
 */
double SwerveModule::GetSpinEncoderRadians()
{
    return spinRelativeEncoder.GetPosition();
}

/**
 *  Returns the current position and rotation of the swerve module, in a WPI struct
 */
SwerveModulePosition SwerveModule::GetSwerveModulePosition()
{
    SwerveModulePosition position = SwerveModulePosition();
    position.distance = units::length::meter_t{GetDriveEncoderMeters()};
    position.angle = Rotation2d(units::degree_t{GetModuleHeading()});
    return position;
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
    spinMotor.Set(0);
    driveMotor.Set(0);
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
    // This could mean moving toward it clockwise, counterclockwise, or moving towards the opposite of the angle
    // and driving in the opposite direction
    if (wheelAngle < targetAngle)
    {
        // if target and wheelangle are less than 90 degrees apart, we should spin directly towards the target angle
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

    // Move motors  at speeds and directions determined earlier
    spinMotor.Set(spinMotorSpeed * spinDirection);
    driveMotor.Set(driveSpeed * driveDirection);
}

/*
  ____                               ____       _
 / ___|_      _____ _ ____   _____  |  _ \ _ __(_)_   _____
 \___ \ \ /\ / / _ \ '__\ \ / / _ \ | | | | '__| \ \ / / _ \
  ___) \ V  V /  __/ |   \ V /  __/ | |_| | |  | |\ V /  __/
 |____/ \_/\_/ \___|_|    \_/ \___| |____/|_|  |_| \_/ \___|

*/

/**
 * Instantiates a swerve drive
 */
SwerveDrive::SwerveDrive()
    : FLModule{FL_DRIVE_MOTOR_PORT, FL_SPIN__MOTOR_PORT, FL_MAGNETIC_ENCODER_PORT, FL_WHEEL_OFFSET},
      FRModule{FR_DRIVE_MOTOR_PORT, FR_SPIN__MOTOR_PORT, FR_MAGNETIC_ENCODER_PORT, FR_WHEEL_OFFSET},
      BLModule{BL_DRIVE_MOTOR_PORT, BL_SPIN__MOTOR_PORT, BL_MAGNETIC_ENCODER_PORT, BL_WHEEL_OFFSET},
      BRModule{BR_DRIVE_MOTOR_PORT, BR_SPIN__MOTOR_PORT, BR_MAGNETIC_ENCODER_PORT, BR_WHEEL_OFFSET},
      pigeonIMU{PIGEON_IMU_PORT},
      FLWheelPos{DRIVE_LENGTH / 2, DRIVE_WIDTH / 2},
      FRWheelPos{DRIVE_LENGTH / 2, -DRIVE_WIDTH / 2},
      BLWheelPos{-DRIVE_LENGTH / 2, DRIVE_WIDTH / 2},
      BRWheelPos{-DRIVE_LENGTH / 2, DRIVE_WIDTH / 2},
      wheelTranslationArray{FLWheelPos, FRWheelPos, BLWheelPos, BRWheelPos},
      kinematics{wheelTranslationArray},
      odometry{kinematics, Rotation2d(0_deg), GetSwerveModulePositions()}
{
}

/**
 * Returns the absolute heading of the swerve drive according the the IMU (gyroscope).
 *
 * @return The Swerve Drive's heading in degrees with 0.0 being the front of the robot increasing clockwise.
 */
double SwerveDrive::GetIMUHeading()
{
    // Turns the degree returned into a number 0-360
    double pigeon_angle = fmod(pigeonIMU.GetYaw().GetValueAsDouble(), 360);

    // Reverses the angle so that positive is clockwise
    pigeon_angle = 360 - pigeon_angle;

    // Fixes weird issues with the number 360
    if (pigeon_angle == 360)
        pigeon_angle = 0;

    return pigeon_angle;
}

/**
 * Resets Robot heading to facing away from the driver
 */
void SwerveDrive::ResetHeading()
{
    Pose2d currentPose = GetOdometryPose();
    ResetOdometry(Pose2d(currentPose.X(), currentPose.Y(), Rotation2d(0_deg)));
}

/**
 * Returns the current position of each swerve module as an array
 */
wpi::array<SwerveModulePosition, 4> SwerveDrive::GetSwerveModulePositions()
{
    wpi::array<SwerveModulePosition, 4> positions = {FLModule.GetSwerveModulePosition(),
                                                     FRModule.GetSwerveModulePosition(),
                                                     BLModule.GetSwerveModulePosition(),
                                                     BRModule.GetSwerveModulePosition()};
    return positions;
}

/**
 * Resets Odometry to (0,0) facing away from the driver
 */
void SwerveDrive::ResetOdometry()
{
    ResetOdometry(Pose2d(0_m, 0_m, Rotation2d(0_rad)));
}

/**
 * Resets Odometry to a given position
 */
void SwerveDrive::ResetOdometry(Pose2d position)
{
    odometry.ResetPosition(Rotation2d(units::degree_t{GetIMUHeading()}),
                            GetSwerveModulePositions(),
                            frc::Pose2d(Pose2d(position.Y(), position.X(), position.Rotation())));
}

/**
 * Finds the Pose of the robot using odometry
 * The math is complicated and WPI does it for me so I don't completely understand it, but if you have questions you can check their documentation
 */
Pose2d SwerveDrive::GetOdometryPose()
{
    Pose2d pose = odometry.GetPose();
    return Pose2d(pose.Y(), pose.X(), pose.Rotation());
}

/**
 * Updates the odometry reading based on change in each swerve module's positions.
 * Must be called every periodic loop for accuracy (once every 20ms or less)
 */
void SwerveDrive::Update()
{
    odometry.Update(units::degree_t{GetIMUHeading()}, GetSwerveModulePositions());
}

/**
 * Converts a meters per second speed to a percent power argument for the falcon motors.
 */
double SwerveDrive::VelocityToPercent(double velocity)
{
    // The following values were found via experimentation:
    // We ran the swerve drive at different percent speeds and graphed their position using a vernier position sensor.
    // Then we found the line of best fit (aka Velocity), and its equation is below.
    // TLDR: We did a mini physics experiment.
    if (velocity > 0)
        return std::max((velocity + 0.0562) / 4.38, 0.0);
    else
        return std::min((velocity - 0.0562) / 4.38, 0.0);
}

/**
 * Converts a percent power argument for the falcon motors to a meters per second speed.
 */
double SwerveDrive::PercentToVelocity(double percent)
{
    // The Reverse of the Equation Above
    if (percent > 0)
        return std::max(4.38 * percent - 0.0562, 0.0) * 2 * M_PI;
    else
        return std::min(4.38 * percent + 0.0562, 0.0) * 2 * M_PI;
}

/**
 * Converts a meters per second speed to a percent power argument for the falcon motors.
 */
double SwerveDrive::AngularVelocityToPercent(double velocity)
{
    // The following values were found via experimentation:
    // We ran the swerve drive at different percent speeds and recorded the amount of time it took to complete 10 rotations
    // Then we found the line of best fit (aka Velocity), and its equation is below.
    // TLDR: We did a mini physics experiment.
    velocity = velocity / 2 / M_PI;
    if (velocity > 0)
        return std::max((velocity + 0.0329) / 1.92, 0.0);
    else
        return std::min((velocity - 0.0329) / 1.92, 0.0);
}

/**
 * Converts a percent power argument for the falcon motors to a meters per second speed.
 */
double SwerveDrive::AngularPercentToVelocity(double percent)
{
    // The Reverse of the Equation Above
    if (percent > 0)
        return std::max(1.92 * percent - 0.0329, 0.0);
    else
        return std::min(1.92 * percent + 0.0329, 0.0);
}

/**
 * Drives the swerve "robot oriented" meaning the FWD drive speed will move the robot in the direction of the front of the robot
 *
 * @param STRAFE_Drive_Speed The speed the robot should move left and right, positive being right, in percentage (0 - 1.0)
 * @param FWD_Drive_Speed The speed the robot should move forward and back, positive being forward, in percentage (0 - 1.0)
 * @param Turn_Speed The speed the robot should turn left and right, positive being clockwise, in percentage (0 - 1.0)
 */
void SwerveDrive::DriveSwervePercentNonFieldOriented(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed)
{

    // If there is no drive input, don't drive the robot and just end the function
    if (FWD_Drive_Speed == 0 && STRAFE_Drive_Speed == 0 && Turn_Speed == 0)
    {
        FLModule.StopSwerveModule();
        FRModule.StopSwerveModule();
        BLModule.StopSwerveModule();
        BRModule.StopSwerveModule();

        return;
    }

    // Determine wheel speeds / wheel target positions
    // Equations explained at:
    // https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
    // After clicking above link press the top download to see how the equations work
    double driveRadius = sqrt(pow(DRIVE_LENGTH.value(), 2) + pow(DRIVE_WIDTH.value(), 2));

    double A = STRAFE_Drive_Speed - Turn_Speed * (DRIVE_LENGTH.value() / driveRadius);
    double B = STRAFE_Drive_Speed + Turn_Speed * (DRIVE_LENGTH.value() / driveRadius);
    double C = FWD_Drive_Speed - Turn_Speed * (DRIVE_WIDTH.value() / driveRadius);
    double D = FWD_Drive_Speed + Turn_Speed * (DRIVE_WIDTH.value() / driveRadius);

    double FR_Target_Angle = atan2(B, C) * 180 / M_PI;
    double FL_Target_Angle = atan2(B, D) * 180 / M_PI;
    double BL_Target_Angle = atan2(A, D) * 180 / M_PI;
    double BR_Target_Angle = atan2(A, C) * 180 / M_PI;

    double FR_Drive_Speed = sqrt(pow(B, 2) + pow(C, 2));
    double FL_Drive_Speed = sqrt(pow(B, 2) + pow(D, 2));
    double BL_Drive_Speed = sqrt(pow(A, 2) + pow(D, 2));
    double BR_Drive_Speed = sqrt(pow(A, 2) + pow(C, 2));

    // If Turn Speed and Drive Speed are both high, the equations above will output a number greater than 1.
    // Below we must scale down all of the drive speeds to make sure we do not tell the motor to go faster
    // than it's max value.
    double max = FR_Drive_Speed;
    if (FL_Drive_Speed > max)
        max = FL_Drive_Speed;
    if (BL_Drive_Speed > max)
        max = BL_Drive_Speed;
    if (BR_Drive_Speed > max)
        max = BR_Drive_Speed;

    if (max > 1)
    {
        FL_Drive_Speed /= max;
        FR_Drive_Speed /= max;
        BL_Drive_Speed /= max;
        BR_Drive_Speed /= max;
    }

    // Make all the motors move
    FLModule.DriveSwerveModulePercent(FL_Drive_Speed, FL_Target_Angle);
    FRModule.DriveSwerveModulePercent(FR_Drive_Speed, FR_Target_Angle);
    BLModule.DriveSwerveModulePercent(BL_Drive_Speed, BL_Target_Angle);
    BRModule.DriveSwerveModulePercent(BR_Drive_Speed, BR_Target_Angle);
}

/**
 * Drives the swerve drive, field oriented (in relation to the driver's pov) with an x y and spin.
 *
 * @param STRAFE_Drive_Speed The speed the robot should move left and right, positive being right, in percentage (0 - 1.0)
 * @param FWD_Drive_Speed The speed the robot should move forward and back, positive being forward, in percentage (0 - 1.0)
 * @param Turn_Speed The speed the robot should turn left and right, positive being clockwise, in percentage (0 - 1.0)
 */
void SwerveDrive::DriveSwervePercent(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed)
{
    // Converts our field oriented speeds to robot oriented, by using trig with the current robot angle.
    double angle = GetOdometryPose().Rotation().Radians().value();
    double oldFwd = FWD_Drive_Speed;
    FWD_Drive_Speed = FWD_Drive_Speed * cos(angle) + STRAFE_Drive_Speed * sin(angle);
    STRAFE_Drive_Speed = -1 * oldFwd * sin(angle) + STRAFE_Drive_Speed * cos(angle);

    DriveSwervePercentNonFieldOriented(STRAFE_Drive_Speed, FWD_Drive_Speed, Turn_Speed);
}

/**
 * Drives the swerve drive, field oriented (in relation to the driver's pov) with an x y and spin.
 *
 * @param STRAFE_Drive_Speed The speed the robot should move left and right, positive being right, in meters per second
 * @param FWD_Drive_Speed The speed the robot should move forward and back, positive being forward, in meters per second
 * @param Turn_Speed The speed the robot should turn left and right, positive being clockwise, in radians per second
 */
void SwerveDrive::DriveSwerveMetersAndRadians(double STRAFE_Drive_Speed, double FWD_Drive_Speed, double Turn_Speed)
{
    DriveSwervePercent(VelocityToPercent(STRAFE_Drive_Speed), VelocityToPercent(FWD_Drive_Speed), AngularVelocityToPercent(Turn_Speed));
}
