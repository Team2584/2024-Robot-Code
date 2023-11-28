#include "Autonomous Functionality/SwerveDriveAutoControl.h"

/**
 * Instantiates a swerve drive autonomous controller for a basic swerve drive
 * 
 * @param swerveDrive A base swerve drive template
 */
SwerveDriveAutonomousController::SwerveDriveAutonomousController(SwerveDrive *swerveDrive) 
    : xPIDController{ODOMETRY_TRANSLATION_KP, ODOMETRY_TRANSLATION_KI, ODOMETRY_TRANSLATION_KD, ODOMETRY_TRANSLATION_KI_MAX, 
                     ODOMETRY_TRANSLATION_MIN_SPEED, ODOMETRY_TRANSLATION_MAX_SPEED, ODOMETRY_TRANSLATION_TOLERANCE, ODOMETRY_TRANSLATION_VELOCITY_TOLERANCE},
      yPIDController{ODOMETRY_TRANSLATION_KP, ODOMETRY_TRANSLATION_KI, ODOMETRY_TRANSLATION_KD, ODOMETRY_TRANSLATION_KI_MAX, 
                     ODOMETRY_TRANSLATION_MIN_SPEED, ODOMETRY_TRANSLATION_MAX_SPEED, ODOMETRY_TRANSLATION_TOLERANCE, ODOMETRY_TRANSLATION_VELOCITY_TOLERANCE},
      rotationPIDController{ODOMETRY_ROTATION_KP, ODOMETRY_ROTATION_KI, ODOMETRY_ROTATION_KD, ODOMETRY_ROTATION_KI_MAX, 
                     ODOMETRY_ROTATION_MIN_SPEED, ODOMETRY_ROTATION_MAX_SPEED, ODOMETRY_ROTATION_TOLERANCE, ODOMETRY_ROTATION_VELOCITY_TOLERANCE}
{
    baseSwerveDrive = swerveDrive;
}

/**
 * Instantiates a swerve drive autonomous controller for a tag-based swerve drive
 * 
 * @param swerveDrive A photon tag swerve drive template
 */
SwerveDriveAutonomousController::SwerveDriveAutonomousController(PhotonTagSwerve *swerveDrive) : SwerveDriveAutonomousController((SwerveDrive*) swerveDrive)  // TODO write comment
{
    photonTagSwerve = swerveDrive;
}

/**
 * Drives the swerve to a pose on the field
 * 
 * @param odometryType The type of data we want to use to estimate our position on the field
 * @param target Our target position on the field
 */
bool SwerveDriveAutonomousController::DriveToPose(OdometryType odometryType, Pose2d target)
{
    double xSpeed = 0, ySpeed = 0, rotationSpeed = 0;
    bool xDone = true, yDone = true, rotationDone = true;

    if (odometryType == OdometryType::PureOdometry)
    {
        xSpeed = xPIDController.Calculate(baseSwerveDrive->GetOdometryPose().X().value(), target.X().value());
        xDone = xPIDController.PIDFinished();
        ySpeed = yPIDController.Calculate(baseSwerveDrive->GetOdometryPose().Y().value(), target.Y().value());
        yDone = yPIDController.PIDFinished();
        rotationSpeed = rotationPIDController.Calculate(baseSwerveDrive->GetOdometryPose().Rotation().Radians().value(), target.Rotation().Radians().value());
        rotationDone = rotationPIDController.PIDFinished();
    }
    else if (odometryType == OdometryType::TagBased)
    {
        // Check if given reference to a photon tag swerve. 
        if (photonTagSwerve == NULL)
        {
            SmartDashboard::PutString("ERROR", "Attemped to read april tag data from a base swerve drive in SwerveDriveAutoController.cpp!");
            return true;
        }

        xSpeed = xPIDController.Calculate(photonTagSwerve->GetTagOdometryPose().X().value(), target.X().value());
        xDone = xPIDController.PIDFinished();
        ySpeed = yPIDController.Calculate(photonTagSwerve->GetTagOdometryPose().Y().value(), target.Y().value());
        yDone = yPIDController.PIDFinished();
        rotationSpeed = rotationPIDController.Calculate(photonTagSwerve->GetTagOdometryPose().Rotation().Radians().value(), target.Rotation().Radians().value());
        rotationDone = rotationPIDController.PIDFinished();
    }

 
    // Debugging info
    SmartDashboard::PutNumber("Pose X Speed", xSpeed);
    SmartDashboard::PutNumber("Pose Y Speed", ySpeed);
    SmartDashboard::PutNumber("Pose Rotation Speed", rotationSpeed);

    SmartDashboard::PutBoolean("Pose X Done", xDone);
    SmartDashboard::PutBoolean("Pose Y Done", yDone);
    SmartDashboard::PutBoolean("Pose Rotation Done", rotationDone);


    if (xDone && yDone && rotationDone)
      return true;

    // Drive swerve at desired speeds
    baseSwerveDrive->DriveSwervePercent(xSpeed, -ySpeed, -rotationSpeed);
    return false;
}