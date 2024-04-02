#include "Autonomous Functionality/SwerveDriveAutoControl.h"

/**
 * Instantiates a swerve drive autonomous controller for a vision based swerve drive
 * 
 * @param swerveDrive A vision based swerve drive template
 */
SwerveDriveAutonomousController::SwerveDriveAutonomousController(VisionSwerve *swerveDrive_) 
    : xPIDController{DTP_TRANSLATION_KP, DTP_TRANSLATION_KI, DTP_TRANSLATION_KD, DTP_TRANSLATION_KI_MAX, 
                     DTP_TRANSLATION_MIN_SPEED, DTP_TRANSLATION_MAX_SPEED, DTP_TRANSLATION_TOLERANCE, DTP_TRANSLATION_VELOCITY_TOLERANCE},
      yPIDController{DTP_TRANSLATION_KP, DTP_TRANSLATION_KI, DTP_TRANSLATION_KD, DTP_TRANSLATION_KI_MAX, 
                     DTP_TRANSLATION_MIN_SPEED, DTP_TRANSLATION_MAX_SPEED, DTP_TRANSLATION_TOLERANCE, DTP_TRANSLATION_VELOCITY_TOLERANCE},
      rotationPIDController{DTP_ROTATION_KP, DTP_ROTATION_KI, DTP_ROTATION_KD, DTP_ROTATION_KI_MAX, 
                     DTP_ROTATION_MIN_SPEED, DTP_ROTATION_MAX_SPEED, DTP_ROTATION_TOLERANCE, DTP_ROTATION_VELOCITY_TOLERANCE},
      trajXPIDController{TRAJECTORY_TRANSLATION_KP, TRAJECTORY_TRANSLATION_KI, TRAJECTORY_TRANSLATION_KD, TRAJECTORY_TRANSLATION_KI_MAX, 
                     TRAJECTORY_TRANSLATION_MIN_SPEED, TRAJECTORY_TRANSLATION_MAX_SPEED, TRAJECTORY_TRANSLATION_TOLERANCE, TRAJECTORY_TRANSLATION_VELOCITY_TOLERANCE},
      trajYPIDController{TRAJECTORY_TRANSLATION_KP, TRAJECTORY_TRANSLATION_KI, TRAJECTORY_TRANSLATION_KD, TRAJECTORY_TRANSLATION_KI_MAX, 
                     TRAJECTORY_TRANSLATION_MIN_SPEED, TRAJECTORY_TRANSLATION_MAX_SPEED, TRAJECTORY_TRANSLATION_TOLERANCE, TRAJECTORY_TRANSLATION_VELOCITY_TOLERANCE},
      trajRotationPIDController{TRAJECTORY_ROTATION_KP, TRAJECTORY_ROTATION_KI, TRAJECTORY_ROTATION_KD, TRAJECTORY_ROTATION_KI_MAX, 
                     TRAJECTORY_ROTATION_MIN_SPEED, TRAJECTORY_ROTATION_MAX_SPEED, TRAJECTORY_ROTATION_TOLERANCE, TRAJECTORY_ROTATION_VELOCITY_TOLERANCE},           
      noteXPIDController{NOTE_X_KP, NOTE_X_KI, NOTE_X_KD, NOTE_X_KI_MAX, 
                     NOTE_X_MIN_SPEED, NOTE_X_MAX_SPEED, NOTE_X_TOLERANCE, NOTE_X_VELOCITY_TOLERANCE},
      noteYPIDController{NOTE_Y_KP, NOTE_Y_KI, NOTE_Y_KD, NOTE_Y_KI_MAX, 
                     NOTE_Y_MIN_SPEED, NOTE_Y_MAX_SPEED, NOTE_Y_TOLERANCE, NOTE_Y_VELOCITY_TOLERANCE},
      noteRotationPIDController{NOTE_ROTATION_KP, NOTE_ROTATION_KI, NOTE_ROTATION_KD, NOTE_ROTATION_KI_MAX,    
                           NOTE_ROTATION_MIN_SPEED, NOTE_ROTATION_MAX_SPEED, NOTE_ROTATION_TOLERANCE, NOTE_ROTATION_VELOCITY_TOLERANCE} 
{
    swerveDrive = swerveDrive_;
    rotationPIDController.EnableContinuousInput(-M_PI, M_PI);
    noteRotationPIDController.EnableContinuousInput(-M_PI, M_PI);
    trajRotationPIDController.EnableContinuousInput(-M_PI, M_PI);
}

/**
 * Calculates the speeds neccesary to drive to a pose
 * 
 * @param poseEstimationType The type of data we want to use to estimate our position on the field
 * @param target Our target position on the field
 * @param speeds The function will fill this array with the calculating speeds [x, y, rotation]
 * @param PIDComplete THe function will fill this array with the finished PID loops [x, y, rotation]
 */
void SwerveDriveAutonomousController::CalculatePIDToPose(PoseEstimationType poseEstimationType, Pose2d target, double speeds[3], bool PIDComplete[3])
{
    if (poseEstimationType == PoseEstimationType::PureOdometry)
    {
        speeds[0] = xPIDController.Calculate(swerveDrive->GetOdometryPose().X().value(), target.X().value());
        PIDComplete[0] = xPIDController.PIDFinished();
        speeds[1] = yPIDController.Calculate(swerveDrive->GetOdometryPose().Y().value(), target.Y().value());
        PIDComplete[1] = yPIDController.PIDFinished();
        speeds[2] = rotationPIDController.Calculate(swerveDrive->GetOdometryPose().Rotation().Radians().value(), target.Rotation().Radians().value());
        PIDComplete[2] = rotationPIDController.PIDFinished();
    }
    else if (poseEstimationType == PoseEstimationType::TagBased)
    {
        speeds[0] = xPIDController.Calculate(swerveDrive->GetTagOdometryPose().X().value(), target.X().value());
        PIDComplete[0] = xPIDController.PIDFinished();
        speeds[1] = yPIDController.Calculate(swerveDrive->GetTagOdometryPose().Y().value(), target.Y().value());
        PIDComplete[1] = yPIDController.PIDFinished();
        speeds[2] = rotationPIDController.Calculate(swerveDrive->GetTagOdometryPose().Rotation().Radians().value(), target.Rotation().Radians().value());
        PIDComplete[2] = rotationPIDController.PIDFinished();
    }
    else if (poseEstimationType == PoseEstimationType::NoteBased)
    {
        speeds[0] = noteXPIDController.Calculate(swerveDrive->GetTagOdometryPose().X().value(), target.X().value());
        PIDComplete[0] = noteXPIDController.PIDFinished();
        speeds[1] = noteYPIDController.Calculate(swerveDrive->GetTagOdometryPose().Y().value(), target.Y().value());
        PIDComplete[1] = noteYPIDController.PIDFinished();
        speeds[2] = noteRotationPIDController.Calculate(swerveDrive->GetTagOdometryPose().Rotation().Radians().value(), target.Rotation().Radians().value());
        PIDComplete[2] = noteRotationPIDController.PIDFinished();
    }
}

/**
 *  This function can be called to restart the PID Controller for a new setpoint.
 */
void SwerveDriveAutonomousController::ResetPIDLoop()
{
    xPIDController.ResetPIDLoop();
    yPIDController.ResetPIDLoop();
    rotationPIDController.ResetPIDLoop();
    trajXPIDController.ResetPIDLoop();
    trajYPIDController.ResetPIDLoop();
    trajRotationPIDController.ResetPIDLoop();
    noteXPIDController.ResetPIDLoop();
    noteYPIDController.ResetPIDLoop();
    noteRotationPIDController.ResetPIDLoop();
}

/**
 * Call this function before the first DriveToPose() call to initialize.
 * 
 * @param poseEstimationType The type of data we use to estimate our position on the field
 */
void SwerveDriveAutonomousController::BeginDriveToPose(PoseEstimationType poseEstimationType)
{
    ResetPIDLoop();
}

/**
 * Drives the swerve to a pose on the field
 * 
 * @param target Our target position on the field
 * 
 * @return returns true when pose has been reached
 */
bool SwerveDriveAutonomousController::DriveToPose(Pose2d target, PoseEstimationType poseEstimationType)
{
    double speeds[3] = {0, 0, 0};
    bool PIDFinished[3] = {false, false, false};
 
    CalculatePIDToPose(poseEstimationType, target, speeds, PIDFinished);

    // Debugging info
    SmartDashboard::PutNumber("Pose X Speed", speeds[0]);
    SmartDashboard::PutNumber("Pose Y Speed", speeds[1]);
    SmartDashboard::PutNumber("Pose Rotation Speed", speeds[2]);

    SmartDashboard::PutBoolean("Pose X Done", PIDFinished[0]);
    SmartDashboard::PutBoolean("Pose Y Done", PIDFinished[1]);
    SmartDashboard::PutBoolean("Pose Rotation Done", PIDFinished[2]);


    // If all PID loops are finished, stop driving the swerve.
    if (PIDFinished[0] && PIDFinished[1] && PIDFinished[2])
    {
        swerveDrive->DriveSwervePercent(0, 0, 0);
        return true;
    }

    // Drive swerve at desired speeds
    if (poseEstimationType == PoseEstimationType::TagBased)
        swerveDrive->DriveSwervePercentTagOriented(speeds[0], speeds[1], speeds[2]);
    else 
        swerveDrive->DriveSwervePercent(speeds[0], speeds[1], speeds[2]);
    return false;
}

bool SwerveDriveAutonomousController::TurnToAngleWhileDriving(double xSpeed, double ySpeed, Rotation2d target, PoseEstimationType poseEstimationType)
{
    double speeds[3] = {0, 0, 0};
    bool PIDFinished[3] = {false, false, false};
 
    CalculatePIDToPose(poseEstimationType, Pose2d(swerveDrive->GetTagOdometryPose().X(), swerveDrive->GetTagOdometryPose().Y(), target), speeds, PIDFinished);

    // Debugging info
    SmartDashboard::PutNumber("Pose Rotation Speed", speeds[2]);
    SmartDashboard::PutBoolean("Pose Rotation Done", PIDFinished[2]);

    if (PoseEstimationType::TagBased == poseEstimationType)
        swerveDrive->DriveSwervePercentTagOriented(xSpeed, ySpeed, speeds[2]);
    else
        swerveDrive->DriveSwervePercent(xSpeed, ySpeed, speeds[2]);
    return PIDFinished[2];
}

/*
 * Reset the Trajectory Queue
 */
void SwerveDriveAutonomousController::ResetTrajectoryQueue()
{
    while (trajectoryQueue.empty() == false)
    {
        trajectoryQueue.pop();
    }
}

/**
 * Loads a trajectory into memory allowing it to be run in an autonomous
 * Run in auton Init
 * 
 * @param trajectoryString the name of the file to load the trajectory from
 */
void SwerveDriveAutonomousController::LoadTrajectory(string trajectoryString)
{
    shared_ptr<pathplanner::PathPlannerPath> path = pathplanner::PathPlannerPath::fromPathFile(trajectoryString);
    trajectoryQueue.push(pathplanner::PathPlannerTrajectory(path, ChassisSpeeds(), path.get()->getPreviewStartingHolonomicPose().Rotation())); // Blank 
}

/**
 * Iterates to next trajectory in queue of trajectories and prepare for trajectory to start
 */
void SwerveDriveAutonomousController::BeginNextTrajectory()
{
    // initializes current trajectory as next trajectory in the queue
    if (trajectoryQueue.size() > 0)
    {
        currentTrajectory = trajectoryQueue.front();
        trajectoryQueue.pop();
    }
    else
    {
        SmartDashboard::PutString("ERROR", "No more trajectories in queue.");
    }

    // Resets timer so we know how long has passed in the current trajectory
    trajectoryTimer.Restart();
    lastFPGATime = Timer::GetFPGATimestamp();
    lastTrajectoryTime = 0_s;

    ResetPIDLoop();
}

/**
 * Drives the swerve to a pose on the field
 * 
 * @param odometryType The type of data we want to use to estimate our position on the field
 * 
 * @return returns true when pose has been reached
 */
bool SwerveDriveAutonomousController::FollowTrajectory(PoseEstimationType poseEstimationType)
{
    /* Find current state of robot in trajectory (i.e. where the robot should be)*/

    units::second_t currentTime = trajectoryTimer.Get(); /* current time in the trajectory */
    pathplanner::PathPlannerTrajectory::State currentState  = currentTrajectory.sample(currentTime); /* current position estimated for the robot in an ideal world*/
    Rotation2d currentHeading = Rotation2d(currentState.getTargetHolonomicPose().Rotation().Radians());

    if (currentHeading.Radians() > 3.14_rad)
        currentHeading = Rotation2d(currentHeading.Radians() - 6.28_rad);
    else if (currentHeading.Radians() < -3.14_rad)
        currentHeading = Rotation2d(currentHeading.Radians() + 6.28_rad);

    bool trajectoryFinished = currentTrajectory.getTotalTime() < currentTime; /* If the trajectory would be finished in the ideal world */

    /* Set Feed Forward Speeds*/

    units::meters_per_second_t xFeedForward, yFeedForward; /* the current velocity estimated for the robot in an ideal world*/
    units::radians_per_second_t rotationFeedForward; /* the current rotational velocity estimated for the robot in an ideal world*/

    xFeedForward = currentState.velocity * currentState.heading.Cos();
    yFeedForward = currentState.velocity * currentState.heading.Sin(); 
    rotationFeedForward = currentState.headingAngularVelocity;

    if (trajectoryFinished)
    {
        xFeedForward = units::meters_per_second_t{0};
        yFeedForward = units::meters_per_second_t{0};
        rotationFeedForward = units::radians_per_second_t{0};
    }

    SmartDashboard::PutNumber("Trajectory FF X", xFeedForward.value());
    SmartDashboard::PutNumber("Trajectory FF Y", yFeedForward.value());
    SmartDashboard::PutNumber("Trajectory FF Rotation", rotationFeedForward.value());
    SmartDashboard::PutNumber("Trajectory Total Time", currentTrajectory.getTotalTime().value());
    SmartDashboard::PutNumber("Trajectory Current Time", currentTime.value());


    /* Set PID Speeds */

    Pose2d targetPose;
    if (trajectoryFinished)
        targetPose = currentTrajectory.getEndState().getTargetHolonomicPose();
    else
        targetPose = Pose2d(currentState.position, currentHeading);
        
    double PIDSpeeds[3] = {0, 0, 0};
    bool PIDLoopsFinished[3] = {false, false, false};

    if (poseEstimationType == PoseEstimationType::PureOdometry)
    {
        PIDSpeeds[0] = trajXPIDController.Calculate(swerveDrive->GetOdometryPose().X().value(), targetPose.X().value());
        PIDLoopsFinished[0] = trajXPIDController.PIDFinished();
        PIDSpeeds[1] = trajYPIDController.Calculate(swerveDrive->GetOdometryPose().Y().value(), targetPose.Y().value());
        PIDLoopsFinished[1] = trajYPIDController.PIDFinished();
        PIDSpeeds[2] = trajRotationPIDController.Calculate(swerveDrive->GetOdometryPose().Rotation().Radians().value(), targetPose.Rotation().Radians().value());
        PIDLoopsFinished[2] = trajRotationPIDController.PIDFinished();
    }
    else 
    {
        PIDSpeeds[0] = trajXPIDController.Calculate(swerveDrive->GetTagOdometryPose().X().value(), targetPose.X().value());
        PIDLoopsFinished[0] = trajXPIDController.PIDFinished();
        PIDSpeeds[1] = trajYPIDController.Calculate(swerveDrive->GetTagOdometryPose().Y().value(), targetPose.Y().value());
        PIDLoopsFinished[1] = trajYPIDController.PIDFinished();
        PIDSpeeds[2] = trajRotationPIDController.Calculate(swerveDrive->GetTagOdometryPose().Rotation().Radians().value(), targetPose.Rotation().Radians().value());
        PIDLoopsFinished[2] = trajRotationPIDController.PIDFinished();
    }

    bool PIDFinished = PIDLoopsFinished[0] && PIDLoopsFinished[1] && PIDLoopsFinished[2];

    SmartDashboard::PutNumber("Current State X", currentState.position.X().value());
    SmartDashboard::PutNumber("Current State Y", currentState.position.Y().value());
    SmartDashboard::PutNumber("Current State Angle", currentHeading.Degrees().value());
    SmartDashboard::PutBoolean("Trajectory Finished", trajectoryFinished);

    SmartDashboard::PutNumber("Target Pose X", targetPose.X().value());
    SmartDashboard::PutNumber("Target Pose Y", targetPose.Y().value());
    SmartDashboard::PutNumber("Target Pose Angle", targetPose.Rotation().Degrees().value());

    SmartDashboard::PutNumber("Trajectory PID X", PIDSpeeds[0]);
    SmartDashboard::PutNumber("Trajectory PID Y", PIDSpeeds[1]);
    SmartDashboard::PutNumber("Trajectory PID Rotation", PIDSpeeds[2]);

    SmartDashboard::PutBoolean("Trajectory X Done", PIDLoopsFinished[0]);
    SmartDashboard::PutBoolean("Trajectory Y Done", PIDLoopsFinished[1]);
    SmartDashboard::PutBoolean("Trajectory Rotation Done", PIDLoopsFinished[2]);
    
    // If the trajectory and all PID loops are finished, stop driving the swerve.
    if (trajectoryFinished && PIDFinished)
    {
        swerveDrive->DriveSwervePercent(0, 0, 0);
        return true;
    }

    /* Drive the Swerve */
    if (poseEstimationType == PoseEstimationType::TagBased)
        swerveDrive->DriveSwerveTagOrientedMetersAndRadians(xFeedForward.value() + PIDSpeeds[0], yFeedForward.value() + PIDSpeeds[1], /*rotationFeedForward.value() + */ PIDSpeeds[2]);
    else
        swerveDrive->DriveSwerveMetersAndRadians(xFeedForward.value() + PIDSpeeds[0], yFeedForward.value() + PIDSpeeds[1], /*rotationFeedForward.value() + */ PIDSpeeds[2]);
    
    return false;
}

bool SwerveDriveAutonomousController::CalcTrajectoryDriveValues(PoseEstimationType poseEstimationType, double scaleFactor, double finalSpeeds[3])
{
     /* Find current state of robot in trajectory (i.e. where the robot should be)*/
    units::second_t timeDiff = Timer::GetFPGATimestamp() - lastFPGATime;
    lastFPGATime = Timer::GetFPGATimestamp();
    units::second_t currentTime = lastTrajectoryTime + units::second_t{timeDiff.value() * scaleFactor}; 
    lastTrajectoryTime = currentTime;
    SmartDashboard::PutNumber("Current Trajectory Time", currentTime.value());

    pathplanner::PathPlannerTrajectory::State currentState  = currentTrajectory.sample(currentTime); /* current position estimated for the robot in an ideal world*/
    Rotation2d currentHeading = Rotation2d(currentState.getTargetHolonomicPose().Rotation().Radians());

    if (currentHeading.Radians() > 3.14_rad)
        currentHeading = Rotation2d(currentHeading.Radians() - 6.28_rad);
    else if (currentHeading.Radians() < -3.14_rad)
        currentHeading = Rotation2d(currentHeading.Radians() + 6.28_rad);

    bool trajectoryFinished = currentTrajectory.getTotalTime() < currentTime; /* If the trajectory would be finished in the ideal world */

    /* Set Feed Forward Speeds*/

    units::meters_per_second_t xFeedForward, yFeedForward; /* the current velocity estimated for the robot in an ideal world*/
    units::radians_per_second_t rotationFeedForward; /* the current rotational velocity estimated for the robot in an ideal world*/

    xFeedForward = currentState.velocity * currentState.heading.Cos();
    yFeedForward = currentState.velocity * currentState.heading.Sin(); 
    rotationFeedForward = currentState.headingAngularVelocity;

    if (trajectoryFinished)
    {
        xFeedForward = units::meters_per_second_t{0};
        yFeedForward = units::meters_per_second_t{0};
        rotationFeedForward = units::radians_per_second_t{0};
    }

    SmartDashboard::PutNumber("Trajectory FF X", xFeedForward.value());
    SmartDashboard::PutNumber("Trajectory FF Y", yFeedForward.value());
    SmartDashboard::PutNumber("Trajectory FF Rotation", rotationFeedForward.value());
    SmartDashboard::PutNumber("Trajectory Total Time", currentTrajectory.getTotalTime().value());
    SmartDashboard::PutNumber("Trajectory Current Time", currentTime.value());


    /* Set PID Speeds */

    Pose2d targetPose;
    if (trajectoryFinished)
        targetPose = currentTrajectory.getEndState().getTargetHolonomicPose();
    else
        targetPose = Pose2d(currentState.position, currentHeading);
        
    double PIDSpeeds[3] = {0, 0, 0};
    bool PIDLoopsFinished[3] = {false, false, false};

    if (poseEstimationType == PoseEstimationType::PureOdometry)
    {
        PIDSpeeds[0] = trajXPIDController.Calculate(swerveDrive->GetOdometryPose().X().value(), targetPose.X().value());
        PIDLoopsFinished[0] = trajXPIDController.PIDFinished();
        PIDSpeeds[1] = trajYPIDController.Calculate(swerveDrive->GetOdometryPose().Y().value(), targetPose.Y().value());
        PIDLoopsFinished[1] = trajYPIDController.PIDFinished();
        PIDSpeeds[2] = trajRotationPIDController.Calculate(swerveDrive->GetOdometryPose().Rotation().Radians().value(), targetPose.Rotation().Radians().value());
        PIDLoopsFinished[2] = trajRotationPIDController.PIDFinished();
    }
    else 
    {
        PIDSpeeds[0] = trajXPIDController.Calculate(swerveDrive->GetTagOdometryPose().X().value(), targetPose.X().value());
        PIDLoopsFinished[0] = trajXPIDController.PIDFinished();
        PIDSpeeds[1] = trajYPIDController.Calculate(swerveDrive->GetTagOdometryPose().Y().value(), targetPose.Y().value());
        PIDLoopsFinished[1] = trajYPIDController.PIDFinished();
        PIDSpeeds[2] = trajRotationPIDController.Calculate(swerveDrive->GetTagOdometryPose().Rotation().Radians().value(), targetPose.Rotation().Radians().value());
        PIDLoopsFinished[2] = trajRotationPIDController.PIDFinished();
    }

    bool PIDFinished = PIDLoopsFinished[0] && PIDLoopsFinished[1] && PIDLoopsFinished[2];

    SmartDashboard::PutNumber("Current State X", currentState.position.X().value());
    SmartDashboard::PutNumber("Current State Y", currentState.position.Y().value());
    SmartDashboard::PutNumber("Current State Angle", currentHeading.Degrees().value());
    SmartDashboard::PutBoolean("Trajectory Finished", trajectoryFinished);

    SmartDashboard::PutNumber("Target Pose X", targetPose.X().value());
    SmartDashboard::PutNumber("Target Pose Y", targetPose.Y().value());
    SmartDashboard::PutNumber("Target Pose Angle", targetPose.Rotation().Degrees().value());

    SmartDashboard::PutNumber("Trajectory PID X", PIDSpeeds[0]);
    SmartDashboard::PutNumber("Trajectory PID Y", PIDSpeeds[1]);
    SmartDashboard::PutNumber("Trajectory PID Rotation", PIDSpeeds[2]);

    SmartDashboard::PutBoolean("Trajectory X Done", PIDLoopsFinished[0]);
    SmartDashboard::PutBoolean("Trajectory Y Done", PIDLoopsFinished[1]);
    SmartDashboard::PutBoolean("Trajectory Rotation Done", PIDLoopsFinished[2]);
    
    // If the trajectory and all PID loops are finished, stop driving the swerve.
    if (trajectoryFinished && PIDFinished)
    {
        finalSpeeds[0] = 0;
        finalSpeeds[1] = 0;
        finalSpeeds[2] = 0;
        return true;
    }

    finalSpeeds[0] = xFeedForward.value() * scaleFactor + PIDSpeeds[0];
    finalSpeeds[1] = yFeedForward.value() * scaleFactor + PIDSpeeds[1];
    finalSpeeds[2] = PIDSpeeds[2];
    return false;
}

void SwerveDriveAutonomousController::BeginDriveToNote()
{
    BeginDriveToPose(PoseEstimationType::NoteBased);

    hasTurnedToNote = false;
}

bool SwerveDriveAutonomousController::TurnToNote()
{

  if (!swerveDrive->NoteInView() || swerveDrive->limelight.GetTargetArea() < LimelightConstants::NoteAreaCutoff)
    {
        swerveDrive->DriveSwervePercent(0,0,0);
        return false;
    }

    Pose2d currentPose = swerveDrive->GetTagOdometryPose();
    Rotation2d noteTargetAngle = currentPose.Rotation() - Rotation2d(units::degree_t{swerveDrive->GetNoteTx()});

    SmartDashboard::PutNumber("Target Note Angle", noteTargetAngle.Radians().value());

    return DriveToPose(Pose2d(currentPose.Translation(), noteTargetAngle), PoseEstimationType::NoteBased); // Drive to current pose but at the target angle
}

bool SwerveDriveAutonomousController::DriveToNote()
{
    SmartDashboard::PutNumber("NoteArea", swerveDrive->limelight.GetTargetArea());
    SmartDashboard::PutBoolean("NoteInView", swerveDrive->NoteInView());
    if (!swerveDrive->NoteInView() || swerveDrive->limelight.GetTargetArea() < LimelightConstants::NoteAreaCutoff)
    {
        swerveDrive->DriveSwervePercent(0,0,0);
        return false;
    }

    if (!hasTurnedToNote || fabs(swerveDrive->GetNoteTx()) > LimelightConstants::NoteTurnDegCutoff.value())
    {
        hasTurnedToNote = TurnToNote(); 
        return false;
    }
    else{
        hasTurnedToNote = true;
    }

    Pose2d currentPose = swerveDrive->GetTagOdometryPose();
    Rotation2d noteTargetAngle = currentPose.Rotation() + Rotation2d(units::degree_t{swerveDrive->GetNoteTx()});

    SmartDashboard::PutNumber("Target Note Angle", noteTargetAngle.Degrees().value());

    double speeds[3] = {0, 0, 0};
    bool PIDFinished[3] = {false, false, false};
 
    CalculatePIDToPose(PoseEstimationType::NoteBased, Pose2d(0_m, 0_m, noteTargetAngle), speeds, PIDFinished);

    // Debugging info
    SmartDashboard::PutNumber("Pose Rotation Speed", speeds[2]);
    SmartDashboard::PutBoolean("Pose Rotation Done", PIDFinished[2]);


    // If all PID loops are finished, stop driving the swerve.
    if (swerveDrive->GetNoteTy() < 0)
    {
        swerveDrive->DriveSwervePercent(0, 0, 0.001);
        return true;
    }

    speeds[0] = noteXPIDController.Calculate(0, -1 * swerveDrive->limelight.GetNoteTy());
    PIDFinished[0] = noteXPIDController.PIDFinished();
    speeds[1] = noteYPIDController.Calculate(0, swerveDrive->limelight.GetNoteTx());
    PIDFinished[1] = noteYPIDController.PIDFinished();

    // Drive swerve at desired speeds
    swerveDrive->DriveSwervePercentNonFieldOriented(speeds[0], 0.0/*speeds[1]*/, speeds[2]);
    return false;
}