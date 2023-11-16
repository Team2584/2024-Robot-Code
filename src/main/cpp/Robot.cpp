// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Constants/DriverConstants.h"

#include "Swerve.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

SwerveDrive *swerveDrive;
XboxController *xbox_Drive;
XboxController *xbox_Drive2;

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  swerveDrive = new SwerveDrive();
  xbox_Drive = new XboxController(0);
  xbox_Drive2 = new XboxController(1);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoNameCustom)
  {
    // Custom Auto goes here
  }
  else
  {
    // Default Auto goes here
  }
}

void Robot::TeleopInit()
{
  swerveDrive->ResetOdometry();
}

void Robot::TeleopPeriodic()
{
  SmartDashboard::PutNumber("FL Drive Encoder", swerveDrive->FLModule.GetDriveEncoder());
  SmartDashboard::PutNumber("FL Drive Encoder Meters", swerveDrive->FLModule.GetDriveEncoderMeters());
  SmartDashboard::PutNumber("Pigeon IMU Heading", swerveDrive->GetIMUHeading());
  SmartDashboard::PutNumber("Odometry X Position", swerveDrive->GetOdometryPose().X().value());
  SmartDashboard::PutNumber("Odometry Y Position", swerveDrive->GetOdometryPose().Y().value());
  SmartDashboard::PutNumber("Odometry Heading", swerveDrive->GetOdometryPose().Rotation().Degrees().value());
  SmartDashboard::PutNumber("FL Module Heading", swerveDrive->FLModule.GetModuleHeading());
  SmartDashboard::PutNumber("FR Module Heading", swerveDrive->FRModule.GetModuleHeading());
  SmartDashboard::PutNumber("BL Module Heading", swerveDrive->BLModule.GetModuleHeading());
  SmartDashboard::PutNumber("BR Module Heading", swerveDrive->BRModule.GetModuleHeading());


  /* UPDATES */

  swerveDrive->Update();

  /* DRIVER INPUT AND CONTROL */

  // Find controller input
  double leftJoystickX, leftJoystickY, rightJoystickX;
  leftJoystickY = xbox_Drive->GetLeftY();
  leftJoystickX = xbox_Drive->GetLeftX();
  rightJoystickX = xbox_Drive->GetRightX();
  leftJoystickY *= -1;

  // Remove ghost movement by making sure joystick is moved a certain amount
  double leftJoystickDistance = sqrt(pow(leftJoystickX, 2.0) + pow(leftJoystickY, 2.0));

  if (leftJoystickDistance < CONTROLLER_DEADBAND)
  {
    leftJoystickX = 0;
    leftJoystickY = 0;
  }

  if (abs(rightJoystickX) < CONTROLLER_DEADBAND)
  {
    rightJoystickX = 0;
  }

  // Scale control values to max speed
  double FwdDriveSpeed = leftJoystickY * MAX_DRIVE_SPEED;
  double StrafeDriveSpeed = leftJoystickX * MAX_DRIVE_SPEED;
  double TurnSpeed = rightJoystickX * MAX_SPIN_SPEED;

  // Drive the robot
  swerveDrive->DriveSwervePercent(FwdDriveSpeed, StrafeDriveSpeed, TurnSpeed);
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
