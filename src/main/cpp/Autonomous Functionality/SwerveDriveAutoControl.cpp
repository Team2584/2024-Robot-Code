#include "Autonomous Functionality/SwerveDriveAutoControl.h"

SwerveDriveAutonomousController::SwerveDriveAutonomousController(SwerveDrive *swerveDrive_) 
    : xPIDController{ODOMETRY_TRANSLATION_KP, ODOMETRY_TRANSLATION_KI, ODOMETRY_TRANSLATION_KD, ODOMETRY_TRANSLATION_KI_MAX, 
                     ODOMETRY_TRANSLATION_MIN_SPEED, ODOMETRY_TRANSLATION_MAX_SPEED, ODOMETRY_TRANSLATION_TOLERANCE, ODOMETRY_TRANSLATION_VELOCITY_TOLERANCE},
      yPIDController{ODOMETRY_TRANSLATION_KP, ODOMETRY_TRANSLATION_KI, ODOMETRY_TRANSLATION_KD, ODOMETRY_TRANSLATION_KI_MAX, 
                     ODOMETRY_TRANSLATION_MIN_SPEED, ODOMETRY_TRANSLATION_MAX_SPEED, ODOMETRY_TRANSLATION_TOLERANCE, ODOMETRY_TRANSLATION_VELOCITY_TOLERANCE},
      rotationPIDController{ODOMETRY_ROTATION_KP, ODOMETRY_ROTATION_KI, ODOMETRY_ROTATION_KD, ODOMETRY_ROTATION_KI_MAX, 
                     ODOMETRY_ROTATION_MIN_SPEED, ODOMETRY_ROTATION_MAX_SPEED, ODOMETRY_ROTATION_TOLERANCE, ODOMETRY_ROTATION_VELOCITY_TOLERANCE}
{
    swerveDrive = swerveDrive_;
}

bool SwerveDriveAutonomousController::DriveToPose(OdometryType odometryType, Pose2d target)
{
    double xSpeed = 0, ySpeed = 0, rotationSpeed = 0;
    bool xDone = true, yDone = true, rotationDone = true;

    if (odometryType == OdometryType::PureOdometry)
    {
        xSpeed = xPIDController.Calculate(swerveDrive->GetOdometryPose().X().value(), target.X().value());
        xDone = xPIDController.PIDFinished();
        ySpeed = yPIDController.Calculate(swerveDrive->GetOdometryPose().Y().value(), target.Y().value());
        yDone = yPIDController.PIDFinished();
        rotationSpeed = rotationPIDController.Calculate(swerveDrive->GetOdometryPose().Rotation().Radians().value(), target.Rotation().Radians().value());
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
    swerveDrive->DriveSwervePercent(0, 0, -rotationSpeed);// -ySpeed, rotationSpeed);
    return false;}