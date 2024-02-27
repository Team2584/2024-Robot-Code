// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <queue>

#include "Tools/PID.h"
#include "Tools/Math.h"

#include <numbers>
#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>

#include <fmt/core.h>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/DigitalInput.h>
#include <frc/DriverStation.h>

#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>

#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <frc/apriltag/AprilTag.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include <wpi/array.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>

#include <rev/CANSparkBase.h>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <rev/AbsoluteEncoder.h>

#include <frc/PWM.h>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/PIDController.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/PathPlannerTrajectory.h>

using namespace frc;
using namespace std;

enum AllianceColor{RED, BLUE};

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoBCSI2S = "BLUE Center: Shoot -> Intake 2 -> Shoot";
  const std::string kAutoBLSI3S = "BLUE Left: Shoot -> Intake 3 -> Shoot";
  const std::string kAutoBRSI1S = "BLUE Right: Shoot -> Intake 1 -> Shoot";
  const std::string kAutoRCSI10S = "RED Center: Shoot -> Intake 10 -> Shoot";
  const std::string kAutoRLSI9S = "RED Left: Shoot -> Intake 9 -> Shoot";
  const std::string kAutoRRSI11S = "RED Right: Shoot -> Intake 11 -> Shoot";
  const std::string kAutoBR4CloseNotes = "BLUE Right: Shoot -> Intake 3 Close Notes -> Shoot";
  const std::string kAutoBLSI3SI8S = "BLUE Left: Shoot -> Shoot 3 -> Shoot 8";
  const std::string kAutoBLSS3S8TEST = "TESTTT BLUE Left: Shoot -> Shoot 3 -> Shoot 8";
  const std::string kAutoFASTTEST = "Fast Blue Right Shoot -> Intake 3 Close Notes -> Shoot";
  const std::string kAutoBR5Note = "Blue Right Shoot -> Intake 3 Close Notes -> Intake 8";
  std::string m_autoSelected;
};
