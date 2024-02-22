#include "Robot.h"
#include "VisionBasedSwerve.h"
#include "Constants/SwerveConstants.h"
#include "Constants/FieldConstants.h"

#ifndef SWERVE_CONTROLLER_H
#define SWERVE_CONTROLLER_H


/**
 * This class inherits from the base SwerveDrive class, but adds the functionality of calculating odometry using the photonvision library, a camera, and april tags
 */
class SwerveDriveAutonomousController
{
private:
    queue<pathplanner::PathPlannerTrajectory> trajectoryQueue; /* Queue of trajectories initialized and prepared to run */
    pathplanner::PathPlannerTrajectory currentTrajectory;  /* current trajectory in use */
    Timer trajectoryTimer; /* tracks time spent in current trajectory*/

    bool hasTurnedToNote = false;
    Rotation2d noteTargetAngle = Rotation2d(0_rad);

public:
    VisionSwerve *swerveDrive; /* A reference to our swerve drive. */
    PID xPIDController, yPIDController, rotationPIDController; /* PID Controllers to move in various directions */
    PID trajXPIDController, trajYPIDController, trajRotationPIDController; /* PID Controllers to follow a trajectory */
    PID noteXPIDController, noteYPIDController, noteRotationPIDController; /* PID Controllers to auto pick up a note */

    SwerveDriveAutonomousController(VisionSwerve *swerveDrive);

private:
    void CalculatePIDToPose(PoseEstimationType poseEstimationType, Pose2d target, double speeds[3], bool PIDsComplete[3]);
    void ResetPIDLoop();

public:
    Pose2d GetTagPose();
    void BeginDriveToPose(PoseEstimationType poseEstimationType);
    bool DriveToPose(Pose2d target, PoseEstimationType poseEstimationType);
    void TurnToAngleWhileDriving(double xSpeed, double ySpeed, Rotation2d target, PoseEstimationType poseEstimationType);
    void ResetTrajectoryQueue();
    void LoadTrajectory(string trajectoryString);
    void BeginNextTrajectory();
    bool FollowTrajectory(PoseEstimationType poseEstimationType);
    void BeginDriveToNote();
    bool TurnToNote();
    bool DriveToNote();
};

#endif