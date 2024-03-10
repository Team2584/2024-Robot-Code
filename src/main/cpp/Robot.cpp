// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Constants/TeleopConstants.h"
#include "Constants/IntakeConstants.h"
#include "Constants/FlywheelConstants.h"

#include "VisionBasedSwerve.h"
#include "Autonomous Functionality/SwerveDriveAutoControl.h"
#include "Autonomous Functionality/SpeakerFunctionality.h"
#include "Autonomous Functionality/AmpFunctionality.h"
#include "Autonomous Functionality/TrapFunctionality.h"
#include "Autonomous Functionality/AutonomousRoutines.h"
#include "Intake.h"
#include "FlyWheel.h"
#include "Elevator.h"
#include "Climb.h"
#include "NoteController.h"
#include "CANdleLED.h"

PowerDistribution m_pdh{34, frc::PowerDistribution::ModuleType::kRev};
VisionSwerve swerveDrive{};
RumbleXboxController xboxController{0};
RumbleXboxController xboxController2{1};
XboxController xboxController3{2};
Intake overbumper{};
FlywheelSystem flywheel{};
Elevator ampmech{};
Climb hang{&swerveDrive};
NoteController notecontroller{&overbumper, &flywheel, &ampmech};
LightsSubsystem lights{&m_pdh};

SwerveDriveAutonomousController swerveAutoController{&swerveDrive};
AutonomousShootingController flywheelController{&swerveAutoController, &flywheel, &overbumper, &ampmech};
AutonomousAmpingController autoAmpController{&swerveAutoController, &notecontroller};
AutonomousTrapController autoTrapController{&notecontroller, &ampmech, &hang, &intake};

AutonomousController autoController{&swerveDrive, &overbumper, &flywheel, &ampmech, &swerveAutoController, &notecontroller, &flywheelController, &autoAmpController};

Elevator::ElevatorSetting elevSetHeight = Elevator::LOW;
Intake::WristSetting wristSetPoint = Intake::HIGH;


AllianceColor allianceColor = AllianceColor::BLUE;

enum DRIVER_MODE {BASIC, AUTO_AIM_STATIONARY, SHOOT_ON_THE_MOVE, AUTO_AMP, AUTO_INTAKE, CLIMBING_TRAP};
DRIVER_MODE currentDriverMode = DRIVER_MODE::BASIC;

Timer shotTimer = Timer{};
bool anglingToSpeaker = false;
bool begunShooting = false;
double flywheelSetpoint = FLYWHEEL_IDLE_RPM;
units::second_t lastTime = 0_s;
double lastX = 0;
double lastY = 0;
double lastRot = 0;

bool lockingAnglerForClimb = false;

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoBCSI2S, kAutoBCSI2S);
  m_chooser.AddOption(kAutoBLSI3S, kAutoBLSI3S);
  m_chooser.AddOption(kAutoBRSI1S, kAutoBRSI1S);
  m_chooser.AddOption(kAutoRCSI10S, kAutoRCSI10S);
  m_chooser.AddOption(kAutoRLSI9S, kAutoRLSI9S);
  m_chooser.AddOption(kAutoRRSI11S, kAutoRRSI11S);
  m_chooser.AddOption(kAutoBRSlow4CloseNotes, kAutoBRSlow4CloseNotes);
  m_chooser.AddOption(kAutoBLSI3SI8S, kAutoBLSI3SI8S);
  m_chooser.AddOption(kAutoBLSS3S8TEST, kAutoBLSS3S8TEST);
  m_chooser.AddOption(kAutoBR4CloseNotes, kAutoBR4CloseNotes);
  m_chooser.AddOption(kAutoBR4CloseNotesAnd8, kAutoBR4CloseNotesAnd8);
  m_chooser.AddOption(kAutoRL4CloseNotesAnd8, kAutoRL4CloseNotesAnd8);
  m_chooser.AddOption(kAutoBC267, kAutoBC267);
  m_chooser.AddOption(kAutoRC1067, kAutoRC1067);
  m_chooser.AddOption(kAutoBR145, kAutoBR145);
  m_chooser.AddOption(kAutoRL945, kAutoRL945);
  m_chooser.AddOption(kAutoBL3267, kAutoBL3267);
  m_chooser.AddOption(kAutoRR111067, kAutoRR111067);
  m_chooser.AddOption(kAutoBR146, kAutoBR146);
  m_chooser.AddOption(kAutoRL946, kAutoRL946);
  m_chooser.AddOption(kAutoBR45, kAutoBR45);
  m_chooser.AddOption(kAutoRL45, kAutoRL45);
  m_chooser.AddOption(kAutoBL387, kAutoBL387);
  m_chooser.AddOption(kAutoRR1187, kAutoRR1187);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  SmartDashboard::PutNumber("Flywheel Setpoint", 0);
  SmartDashboard::PutNumber("Angler Setpoint", M_PI / 2);

  lights.FullClear();

}


/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  lights.UpdateSubsystemLEDS();
  swerveDrive.UpdateRaspiConnection();
  

  if (DriverStation::GetAlliance() == DriverStation::kRed)
    allianceColor = AllianceColor::RED;
  else
    allianceColor = AllianceColor::BLUE;

  SmartDashboard::PutBoolean("In Match", DriverStation::GetMatchType() != DriverStation::MatchType::kNone);
  SmartDashboard::PutBoolean("Is Blue Alliance", allianceColor == AllianceColor::BLUE);
}

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

  ampmech.ResetElevatorEncoder();  

  if (m_autoSelected == kAutoBCSI2S)
  {
    autoController.SetupBasicShootIntakeShoot(Pose2d(1.39_m, 5.51_m, Rotation2d(180_deg)), "BCTo2"); 
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoBLSI3S)
  {
    autoController.SetupBasicShootIntakeShoot(Pose2d(0.76_m, 6.68_m, Rotation2d(-120_deg)), "BLTo3"); 
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoBRSI1S)
  {
    autoController.SetupBasicShootIntakeShoot(Pose2d(0.74_m, 4.35_m, Rotation2d(120_deg)), "BRTo1"); 
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoRLSI9S)
  {
    autoController.SetupBasicShootIntakeShoot(Pose2d(15.81_m, 4.43_m, Rotation2d(60_deg)), "RLTo9"); 
    allianceColor = AllianceColor::RED;
  }
  else if (m_autoSelected == kAutoRCSI10S)
  {
    autoController.SetupBasicShootIntakeShoot(Pose2d(15.22_m, 5.57_m, Rotation2d(0_deg)), "RCTo10"); 
    allianceColor = AllianceColor::RED;
  }
  else if (m_autoSelected == kAutoRRSI11S)
  {
    autoController.SetupBasicShootIntakeShoot(Pose2d(15.79_m, 6.7_m, Rotation2d(150_deg)), "RRTo11"); 
    allianceColor = AllianceColor::RED;
  }
  else if (m_autoSelected == kAutoBLSI3SI8S)
  {
    autoController.SetupBlueLeftShootIntake3ShootIntake8Shoot();
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoBLSS3S8TEST)
  {
    autoController.SetupBlueLeftShootIntake3ShootIntake8ShootTESTING();    
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoBRSlow4CloseNotes)
  {
    autoController.SetupSlowBlueRightShootIntake1ShootIntake2ShootIntake3();
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoBR4CloseNotes)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(1.06_m, 4.35_m, Rotation2d(138.81_deg)), "BRTo1To2To3", 100_m);
    allianceColor = AllianceColor::BLUE;  
  }
  else if (m_autoSelected == kAutoBR4CloseNotesAnd8)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(1.06_m, 4.35_m, Rotation2d(138.81_deg)), "BRTo1To2To3To8", 2.5_m);
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoRL4CloseNotesAnd8)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(15.48_m, 4.4_m, Rotation2d(34.93_deg)), "RLTo9To10To11To8", 2.5_m);
    allianceColor = AllianceColor::RED;
  }
  else if (m_autoSelected == kAutoBC267)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(1.38_m, 5.51_m, Rotation2d(180_deg)), "BCTo2To6To7", 2.5_m);
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoRC1067)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(15.19_m, 5.59_m, Rotation2d(0.81_deg)), "RCTo10To6To7", 2.5_m);
    allianceColor = AllianceColor::RED;
  }
  else if (m_autoSelected == kAutoBR145)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(1.22_m, 4.47_m, Rotation2d(-98.13_deg)), "BRTo1To4To5", 2.5_m);
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoRL945)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(1.06_m, 4.35_m, Rotation2d(138.81_deg)), "RLTo9To4To5", 2.5_m);
    allianceColor = AllianceColor::RED;
  }
  else if (m_autoSelected == kAutoBL3267)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(1.18_m, 6.7_m, Rotation2d(-145.78_deg)), "BLTo3To2To6To7", 2.5_m);
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoRR111067)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(15.34_m, 6.58_m, Rotation2d(83.66_deg)), "RRTo11To10To6To7", 2.5_m);
    allianceColor = AllianceColor::RED;
  }
  else if (m_autoSelected == kAutoBR146)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(1.29_m, 4.47_m, Rotation2d(150.75_deg)), "BRTo1To4To6", 2.5_m);
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoRL946)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(15.37_m, 4.59_m, Rotation2d(27.9_deg)), "RLTo9To4To6", 2.5_m);
    allianceColor = AllianceColor::RED;
  }
  else if (m_autoSelected == kAutoBR45)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(1.1_m, 4.37_m, Rotation2d(137.29_deg)), "BRTo4To5", 2.5_m);
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoRL45)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(15.35_m, 4.46_m, Rotation2d(37.57_deg)), "RLTo4To5", 2.5_m);
    allianceColor = AllianceColor::RED;
  }
  else if (m_autoSelected == kAutoBL387)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(1.01_m, 6.73_m, Rotation2d(-144.64_deg)), "BLTo3To8To7", 2.5_m);
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoRR1187)
  {
    autoController.SetupFollowTrajectoryAndShoot(Pose2d(15.35_m, 6.64_m, Rotation2d(-34.59_deg)), "RRTo11To8To7", 2.5_m);
    allianceColor = AllianceColor::RED;
  }
}

void Robot::AutonomousPeriodic()
{
  /* Updates */
  swerveDrive.Update();

   /* Autos */
  if (m_autoSelected == kAutoBCSI2S)
    autoController.BasicShootIntakeShoot(AllianceColor::BLUE);
  else if (m_autoSelected == kAutoBLSI3S)
    autoController.BasicShootIntakeShoot(AllianceColor::BLUE);
  else if (m_autoSelected == kAutoBRSI1S)
    autoController.BasicShootIntakeShoot(AllianceColor::BLUE);
  else if (m_autoSelected == kAutoRCSI10S)
    autoController.BasicShootIntakeShoot(AllianceColor::RED);
  else if (m_autoSelected == kAutoRLSI9S)
    autoController.BasicShootIntakeShoot(AllianceColor::RED);
  else if (m_autoSelected == kAutoRRSI11S)
    autoController.BasicShootIntakeShoot(AllianceColor::RED);
  else if (m_autoSelected == kAutoBLSI3SI8S)
    autoController.BlueLeftShootIntake3ShootIntake8Shoot();
  else if (m_autoSelected == kAutoBLSS3S8TEST)
    autoController.BlueLeftShootIntake3ShootIntake8ShootTESTING(); 
  else if (m_autoSelected == kAutoBRSlow4CloseNotes)
    autoController.SlowBlueRightShootIntake1ShootIntake2ShootIntake3();
  else if (m_autoSelected == kAutoBR4CloseNotes)
    autoController.FollowTrajectoryAndShoot(AllianceColor::BLUE);
  else if (m_autoSelected == kAutoBR4CloseNotesAnd8)
    autoController.FollowTrajectoryAndShoot(AllianceColor::BLUE);
  else if (m_autoSelected == kAutoRL4CloseNotesAnd8)
    autoController.FollowTrajectoryAndShoot(AllianceColor::RED);
  else if (m_autoSelected == kAutoBC267)
      autoController.FollowTrajectoryAndShoot(AllianceColor::BLUE);
  else if (m_autoSelected == kAutoRC1067)
      autoController.FollowTrajectoryAndShoot(AllianceColor::RED);
  else if (m_autoSelected == kAutoBR145)
      autoController.FollowTrajectoryAndShoot(AllianceColor::BLUE);
  else if (m_autoSelected == kAutoRL945)
      autoController.FollowTrajectoryAndShoot(AllianceColor::RED);
  else if (m_autoSelected == kAutoBL3267)
      autoController.FollowTrajectoryAndShoot(AllianceColor::BLUE);
  else if (m_autoSelected == kAutoRR111067)
      autoController.FollowTrajectoryAndShoot(AllianceColor::RED);
  else if (m_autoSelected == kAutoBR146)
      autoController.FollowTrajectoryAndShoot(AllianceColor::BLUE);
  else if (m_autoSelected == kAutoRL946)
      autoController.FollowTrajectoryAndShoot(AllianceColor::RED);
  else if (m_autoSelected == kAutoBR45)
      autoController.FollowTrajectoryAndShoot(AllianceColor::BLUE);
  else if (m_autoSelected == kAutoRL45)
      autoController.FollowTrajectoryAndShoot(AllianceColor::RED);
  else if (m_autoSelected == kAutoBL387)
      autoController.FollowTrajectoryAndShoot(AllianceColor::BLUE);
  else if (m_autoSelected == kAutoRR1187)
      autoController.FollowTrajectoryAndShoot(AllianceColor::RED);
}

void Robot::TeleopInit()
{
  currentDriverMode = DRIVER_MODE::BASIC;
  begunShooting = false;
  flywheelSetpoint = FLYWHEEL_IDLE_RPM;
  lastTime = Timer::GetFPGATimestamp();
  lastX = 0;
  lastY = 0;
  lastRot = 0;
}

void Robot::TeleopPeriodic()
{
  /* UPDATES */

  swerveDrive.Update();

  /* Controller Data */

  // Find controller input (*-1 converts values to fwd/left/counterclockwise positive)
  double leftJoystickX, leftJoystickY, rightJoystickX, rightJoystickY;
  leftJoystickY = xboxController.GetLeftY();
  leftJoystickX = xboxController.GetLeftX();
  rightJoystickX = xboxController.GetRightX() * -1;
  rightJoystickY = xboxController.GetRightY() * -1;

  // Remove ghost movement by making sure joystick is moved a certain amount
  double leftJoystickDistance = sqrt(pow(leftJoystickX, 2.0) + pow(leftJoystickY, 2.0));
  double rightJoystickDistance = sqrt(pow(rightJoystickX, 2.0) + pow(rightJoystickY, 2.0));

  if (leftJoystickDistance < CONTROLLER_DEADBAND)
  {
    leftJoystickX = 0;
    leftJoystickY = 0;
  }

  if (abs(rightJoystickDistance) < CONTROLLER_DEADBAND)
  {
    rightJoystickX = 0;
    rightJoystickY = 0;
  }

  // Slew rate limit joystics
  if (leftJoystickY > lastX + DRIVE_SLEW_RATE * (Timer::GetFPGATimestamp() - lastTime).value())
    leftJoystickY = lastX + DRIVE_SLEW_RATE * (Timer::GetFPGATimestamp() - lastTime).value();
  else if (leftJoystickY < lastX - DRIVE_SLEW_RATE * (Timer::GetFPGATimestamp() - lastTime).value())
      leftJoystickY = lastX - DRIVE_SLEW_RATE * (Timer::GetFPGATimestamp() - lastTime).value();
 
  if (leftJoystickX > lastY + DRIVE_SLEW_RATE * (Timer::GetFPGATimestamp() - lastTime).value())
    leftJoystickX = lastY + DRIVE_SLEW_RATE * (Timer::GetFPGATimestamp() - lastTime).value();
  else if (leftJoystickX < lastY - DRIVE_SLEW_RATE * (Timer::GetFPGATimestamp() - lastTime).value())
      leftJoystickX = lastY - DRIVE_SLEW_RATE * (Timer::GetFPGATimestamp() - lastTime).value();
 
 if (rightJoystickX > lastRot + SPIN_SLEW_RATE * (Timer::GetFPGATimestamp() - lastTime).value())
    rightJoystickX = lastRot + SPIN_SLEW_RATE * (Timer::GetFPGATimestamp() - lastTime).value();
  else if (rightJoystickX < lastRot - SPIN_SLEW_RATE * (Timer::GetFPGATimestamp() - lastTime).value())
      rightJoystickX = lastRot - SPIN_SLEW_RATE * (Timer::GetFPGATimestamp() - lastTime).value();

  lastX = leftJoystickY;
  lastY = leftJoystickX;
  lastRot = rightJoystickX;
  lastTime = Timer::GetFPGATimestamp();

  // Find controller input (*-1 converts values to fwd/left/counterclockwise positive)
  double controller2LeftJoystickX, controller2LeftJoystickY, controller2RightJoystickX, controller2RightJoystickY;
  controller2LeftJoystickY = xboxController2.GetLeftY() * -1;
  controller2LeftJoystickX = xboxController2.GetLeftX() * -1;
  controller2RightJoystickX = xboxController2.GetRightX() * -1;
  controller2RightJoystickY = xboxController2.GetRightY();

  // Remove ghost movement by making sure joystick is moved a certain amount
  double controller2leftJoystickDistance = sqrt(pow(controller2LeftJoystickX, 2.0) + pow(controller2LeftJoystickY, 2.0));
  double controller2rightJoystickDistance = sqrt(pow(controller2RightJoystickX, 2.0) + pow(controller2RightJoystickY, 2.0));

  if (controller2leftJoystickDistance < CONTROLLER_DEADBAND)
  {
    controller2LeftJoystickX = 0;
    controller2LeftJoystickY = 0;
  }

  if (abs(controller2rightJoystickDistance) < CONTROLLER_DEADBAND)
  {
    controller2RightJoystickX = 0;
    controller2RightJoystickY = 0;
  }

  /*                                               
  ,---.                                            
  '   .-' ,--.   ,--. ,---. ,--.--.,--.  ,--.,---.  
  `.  `-. |  |.'.|  || .-. :|  .--' \  `'  /| .-. : 
  .-'    ||   .'.   |\   --.|  |     \    / \   --. 
  `-----' '--'   '--' `----'`--'      `--'   `----'
  */

  switch (currentDriverMode) 
  {
    case DRIVER_MODE::BASIC:
    {
      // SWERVE

      double fwdDriveSpeed = leftJoystickY * MAX_DRIVE_SPEED;
      double strafeDriveSpeed = leftJoystickX * MAX_DRIVE_SPEED;
      double turnSpeed = rightJoystickX * MAX_SPIN_SPEED;

      // SPEED BOOST
      if (xboxController.GetRightBumper())
      {
        fwdDriveSpeed = leftJoystickY * SPEED_BOOST_DRIVE;
        strafeDriveSpeed = leftJoystickX * SPEED_BOOST_DRIVE;
        turnSpeed = rightJoystickX * SPEED_BOOST_SPIN;      
      }

      if (allianceColor == AllianceColor::BLUE)
      {
        fwdDriveSpeed *= -1;
        strafeDriveSpeed *= -1;
      }

      // Reset Swerve Heading and Elevator Encoder
      if ((xboxController.GetStartButton() && xboxController.GetBackButtonPressed()) || (xboxController.GetStartButtonPressed() && xboxController.GetBackButton()))
      {
        ampmech.ResetElevatorEncoder();
        if(allianceColor == AllianceColor::BLUE)
          swerveDrive.ResetOdometry(Pose2d(0_m, 0_m, Rotation2d(180_deg)));
        else
          swerveDrive.ResetOdometry(Pose2d(0_m, 0_m, Rotation2d(0_deg)));
      }

      // Drive Swerve and Lock to a certain cardinal direction if x or y is pressed
      if (xboxController.GetXButton())
      {
        Rotation2d target;
        if (swerveDrive.GetOdometryPose().Rotation().Degrees() < 0_deg)
          target = -90_deg;
        else
          target = 90_deg;
        swerveAutoController.TurnToAngleWhileDriving(fwdDriveSpeed, strafeDriveSpeed, target, PoseEstimationType::PureOdometry);
      }
      else if (xboxController.GetYButton())
      {
        Rotation2d target;
        if (swerveDrive.GetOdometryPose().Rotation().Degrees() < 90_deg && swerveDrive.GetOdometryPose().Rotation().Degrees() > -90_deg)
          target = 0_deg;
        else
          target = 180_deg;
        swerveAutoController.TurnToAngleWhileDriving(fwdDriveSpeed, strafeDriveSpeed, target, PoseEstimationType::PureOdometry);
      }
      else
      {
        swerveDrive.DriveSwervePercent(fwdDriveSpeed, strafeDriveSpeed, turnSpeed);
      }

      // Lights
      if(overbumper.GetObjectInIntake()){
        lights.SetHaveNote();
      }
      else{
        lights.NoLongerHaveNote();
        lights.SetDriving();
      }

      // Note Controller Stuff
      wristSetPoint = Intake::SHOOT;
      
      // These function calls "prepare" the true function calls below
      if ((xboxController2.GetRightBumperPressed() && !xboxController2.GetAButton()) || (xboxController2.GetAButtonPressed() && !xboxController2.GetRightBumper())){
        notecontroller.BeginScoreNoteInPosition(Elevator::ElevatorSetting::AMP);
      }
      if (!begunShooting && xboxController2.GetRightTriggerAxis() > TRIGGER_ACTIVATION_POINT)
      {
        begunShooting = true;
        overbumper.BeginShootNote();
      }
      else if (begunShooting && xboxController2.GetRightTriggerAxis() < TRIGGER_DEACTIVATION_POINT)
      {
        flywheelSetpoint = FLYWHEEL_IDLE_RPM;
        begunShooting = false;
      }

      if (xboxController.GetRightTriggerAxis() > TRIGGER_ACTIVATION_POINT)
      {
        bool done = notecontroller.IntakeNoteToSelector();
        if (!done){
          wristSetPoint = Intake::LOW;
        }
        else {
          xboxController.HaveNoteRumble();
        }
      }
      else if (xboxController.GetLeftTriggerAxis() > TRIGGER_ACTIVATION_POINT)
      {
        overbumper.OuttakeNote();
      }
      else if(xboxController2.GetBButton()){
        notecontroller.ToElevator();
      }
      else if(xboxController2.GetRightTriggerAxis() > TRIGGER_ACTIVATION_POINT){
        overbumper.ShootNote();
      }
      else if (xboxController2.GetRightBumper())
      {
        notecontroller.ScoreNoteInPosition(Elevator::ElevatorSetting::AMP);
      }
      else if (xboxController2.GetAButton())
      {
        notecontroller.LiftNoteToPosition(Elevator::ElevatorSetting::AMP);
      }
      else if (controller2LeftJoystickY != 0)
      {
        overbumper.SetIntakeMotorSpeed(0);
        ampmech.SetAmpMotorPercent(0);
        ampmech.MoveElevatorPercent(controller2LeftJoystickY);
      }
      else {
        overbumper.SetIntakeMotorSpeed(0);
        ampmech.SetAmpMotorPercent(0);
        ampmech.MoveToHeight(Elevator::ElevatorSetting::LOW);
      }

      overbumper.PIDWristToPoint(wristSetPoint);


      double anglerSetpoint = 0.8;
      // Spin up flywheel to various presets
      if (xboxController2.GetStartButtonPressed())
        flywheelSetpoint = 4000;
      else if(xboxController2.GetXButtonPressed())
        flywheelSetpoint = 4000;
      else if (xboxController2.GetYButtonPressed())
        flywheelSetpoint = 4500;

      if (xboxController2.GetXButton())
        anglerSetpoint = 0.8;
      else if (xboxController2.GetYButton())
        anglerSetpoint = 0.65;
      else
        anglerSetpoint = 0.8;

      if (flywheel.TopFlywheel.GetMeasurement() * 60.0 > flywheelSetpoint || flywheel.BottomFlywheel.GetMeasurement() * 60.0 > flywheelSetpoint)
        flywheel.SpinFlywheelPercent(0);
      else
        flywheel.SetFlywheelVelocity(flywheelSetpoint);


      if(xboxController.GetRightTriggerAxis() > TRIGGER_ACTIVATION_POINT || xboxController2.GetBackButtonPressed()){
        lockingAnglerForClimb = false;
      }

      if(!lockingAnglerForClimb){
        flywheel.PIDAngler(anglerSetpoint); 
      }
      else{
        flywheel.PIDAngler(1.399);
      }

      if(xboxController2.GetStartButtonPressed()){
        hang.climbZeroed = false;
      }

      //hang.SetClimbMotors(controller2LeftJoystickY, controller2RightJoystickY);

      // Switching Driver Mode
      if (xboxController2.GetLeftBumperPressed())
      {
        flywheelController.BeginAimAndFire(allianceColor);
        currentDriverMode = DRIVER_MODE::AUTO_AIM_STATIONARY;
      }
      if (xboxController.GetLeftBumperPressed()){
        autoAmpController.BeginDriveToAmp(allianceColor);
        notecontroller.BeginScoreNoteInPosition(Elevator::ElevatorSetting::AMP);
        currentDriverMode = DRIVER_MODE::AUTO_AMP;
      }
      if (xboxController.GetBButtonPressed()){
        swerveAutoController.BeginDriveToNote();
        currentDriverMode = DRIVER_MODE::AUTO_INTAKE;
      }
      if (xboxController2.GetLeftTriggerAxis() > TRIGGER_ACTIVATION_POINT)
      {
        notecontroller.BeginScoreNoteInPosition(Elevator::ElevatorSetting::AMP);
        flywheelController.BeginAimAndFire(allianceColor);
        currentDriverMode = DRIVER_MODE::SHOOT_ON_THE_MOVE;
      }
      if(xboxController2.GetPOV() != -1 || xboxController.GetPOV() != -1){
        currentDriverMode = DRIVER_MODE::CLIMBING_TRAP;
      }

      break;
    }

    case DRIVER_MODE::AUTO_AIM_STATIONARY:
    {
      bool doneShooting = flywheelController.AimAndFire(allianceColor);
    
      overbumper.PIDWristToPoint(Intake::SHOOT);

      if(doneShooting){
        lights.SetStrobeGreen();
        xboxController.ShotNoteRumble();
      }
      else{
        lights.SetFadeOrange();
      }

      if (!xboxController2.GetLeftBumper() || doneShooting)
      {
        flywheelSetpoint = FLYWHEEL_IDLE_RPM;
        currentDriverMode = DRIVER_MODE::BASIC;
      }
    
      break;
    }

    case DRIVER_MODE::AUTO_INTAKE:
    {
      swerveAutoController.DriveToNote();
      bool done = notecontroller.IntakeNoteToSelector();
      if (!done)
      {
        overbumper.PIDWristToPoint(Intake::WristSetting::LOW);
      }
      else
      {
        overbumper.PIDWristToPoint(Intake::WristSetting::HIGH);
        xboxController.HaveNoteRumble();
      }

      if(done){
        xboxController.HaveNoteRumble();
        lights.SetStrobeGreen();
      }
      else{
        lights.SetFadeOrange();
      }

      if (!xboxController.GetBButton() || done)
        currentDriverMode = DRIVER_MODE::BASIC;

      break;
    }

    case DRIVER_MODE::SHOOT_ON_THE_MOVE:
    {
      double fwdDriveSpeed = leftJoystickY * MAX_DRIVE_SPEED_SHOOT_ON_THE_MOVE;
      double strafeDriveSpeed = leftJoystickX * MAX_DRIVE_SPEED_SHOOT_ON_THE_MOVE;

      if (allianceColor == AllianceColor::BLUE)
      {
        fwdDriveSpeed *= -1;
        strafeDriveSpeed *= -1;
      }

      bool turnt = flywheelController.TurnToSpeakerWhileDriving(fwdDriveSpeed, strafeDriveSpeed, allianceColor);
      bool spinning = flywheelController.SpinFlywheelForSpeaker(allianceColor);
      bool angled = flywheelController.AngleFlywheelToSpeaker(allianceColor);
      bool cleared = flywheelController.ClearElevatorForShot();

      if (!begunShooting && xboxController2.GetRightTriggerAxis() > TRIGGER_ACTIVATION_POINT)
      {
        begunShooting = true;
        overbumper.BeginShootNote();
        shotTimer.Restart();
      }
      else if (begunShooting && xboxController2.GetRightTriggerAxis() < TRIGGER_DEACTIVATION_POINT)
      {
        begunShooting = false;
      }

      if(xboxController2.GetRightTriggerAxis() > TRIGGER_ACTIVATION_POINT)
        overbumper.ShootNote();
      else
        overbumper.SetIntakeMotorSpeed(0);

      
      if (turnt && spinning && cleared){
        xboxController2.ReadyActionRumble();
        lights.SetStrobeBlue();
      }
      else{
        lights.SetFadeOrange();
      }
        
      if (xboxController2.GetLeftTriggerAxis() < TRIGGER_DEACTIVATION_POINT || (shotTimer.Get() > SHOT_TIME && begunShooting))
      {
        flywheelSetpoint = FLYWHEEL_IDLE_RPM;
        currentDriverMode = DRIVER_MODE::BASIC;
      }

      break;
    }

    case DRIVER_MODE::AUTO_AMP:
    {
      bool isAtAmp = autoAmpController.DriveToAmp(allianceColor);
      bool scored = false;

      if ((xboxController2.GetRightBumperPressed() && !xboxController2.GetAButton()) || (xboxController2.GetAButtonPressed() && !xboxController2.GetRightBumper()))
        notecontroller.BeginScoreNoteInPosition(Elevator::ElevatorSetting::AMP);

      if (xboxController2.GetRightBumper())
      {
        scored = notecontroller.ScoreNoteInPosition(Elevator::ElevatorSetting::AMP);
      }
      else if (xboxController2.GetAButton())
      {
        notecontroller.LiftNoteToPosition(Elevator::ElevatorSetting::AMP);
      }
      else {
        overbumper.SetIntakeMotorSpeed(0);
        ampmech.SetAmpMotorPercent(0);
        ampmech.MoveToHeight(Elevator::ElevatorSetting::LOW);
      }


      // Complete Auto Amping Code (takes control of amp mech from tyler)

      /*if(isAtAmp || xboxController2.GetRightBumper()){ 
        scored = notecontroller.ScoreNoteInPosition(Elevator::ElevatorSetting::AMP);
      }
      else{
        notecontroller.LiftNoteToPosition(Elevator::ElevatorSetting::AMP);
      }*/

      if(scored){
        xboxController.ShotNoteRumble();
        lights.SetStrobeGreen();
      }
      else if(isAtAmp){
        lights.SetStrobeBlue();
        xboxController2.ReadyActionRumble();
      }
      else{
        lights.SetFadeOrange();
      }
      
      if(!xboxController.GetLeftBumper() || scored){
        currentDriverMode = DRIVER_MODE::BASIC;
      }

      break;
    }

    case DRIVER_MODE::CLIMBING_TRAP:
    {

      lights.SetClimbing();

      double fwdDriveSpeed = leftJoystickY * MAX_DRIVE_SPEED_CLIMB;
      double strafeDriveSpeed = leftJoystickX * MAX_DRIVE_SPEED_CLIMB;
      double turnSpeed = rightJoystickX * MAX_SPIN_SPEED_CLIMB;

      swerveDrive.DriveSwervePercent(fwdDriveSpeed, strafeDriveSpeed, turnSpeed);

      lockingAnglerForClimb = true;
      flywheel.PIDAngler(1.399);

      if(xboxController2.GetPOV() == 180){
        hang.ZeroClimb();
      } 
      else if(xboxController.GetPOV() == 0){
        hang.ExtendClimb();
      }
      else if(xboxController.GetPOV() == 180){
        hang.RetractClimb();
      }
      else if(xboxController.GetPOV() == 90 || xboxController.GetPOV() == 270){
        if(xboxController.GetPOV() == 90){hang.leftClimbMotor.Set(ClimbConstants::BasePctDown*-1);}
        if(xboxController.GetPOV() == 270){hang.rightClimbMotor.Set(ClimbConstants::BasePctDown);}
      }
      else if(xboxController2.GetPOV() == 90){
        //autoTrapController.PrepareClimb();
        hang.BalanceWhileClimbing();
      }
      else if (xboxController2.GetPOV() == 270){
         if(autoTrapController.ClimbToTrap()){
            notecontroller.ScoreNoteInPosition(Elevator::ElevatorSetting::TRAP);
         }
      }
      else if (controller2LeftJoystickY != 0)
      {
        ampmech.MoveElevatorPercent(controller2LeftJoystickY);
        hang.HoldClimb();
      }
      else{
        hang.HoldClimb();
      }

      if (xboxController2.GetRightBumper())
        ampmech.DepositNote();
      else
        ampmech.SetAmpMotorPercent(0);

      overbumper.SetIntakeMotorSpeed(0);

      if (xboxController.GetPOV() == -1 && xboxController2.GetPOV() == -1)
        currentDriverMode = DRIVER_MODE::BASIC;

    }
  }


  /* TESTINGGGGG */

  // Drive to a position for testing
  if (xboxController3.GetYButtonPressed())
    autoAmpController.BeginDriveToAmp(allianceColor);
  if (xboxController3.GetYButton())
    autoAmpController.DriveToAmp(allianceColor);
      
  if (xboxController3.GetXButton()){
    flywheelController.TurnToSpeaker(allianceColor);
  }
  
  // Follow spline for testing
  if (xboxController3.GetBButtonPressed())
  {
    swerveAutoController.ResetTrajectoryQueue();
    swerveAutoController.LoadTrajectory("BRTo1To2To3");
    swerveAutoController.BeginNextTrajectory();
  }
  if (xboxController3.GetAButton())
  {
    double finalSpeeds[2];
    swerveAutoController.CalcTrajectoryDriveValues(PoseEstimationType::TagBased, 0.25, finalSpeeds);
    swerveDrive.DriveSwerveTagOrientedMetersAndRadians(finalSpeeds[0], finalSpeeds[1], finalSpeeds[2]);
  }
  else if (xboxController3.GetBButton())
  {
    double finalSpeeds[2];
    swerveAutoController.CalcTrajectoryDriveValues(PoseEstimationType::TagBased, 1, finalSpeeds);
    swerveDrive.DriveSwerveTagOrientedMetersAndRadians(finalSpeeds[0], finalSpeeds[1], finalSpeeds[2]);
  }

  // Follow spline for testing
  /*if (xboxController3.GetAButtonPressed())
  {
    swerveAutoController.ResetTrajectoryQueue();
    swerveAutoController.LoadTrajectory("Test");
    swerveAutoController.BeginNextTrajectory();
  }
  if (xboxController3.GetAButton())
  {
    swerveAutoController.FollowTrajectory(PoseEstimationType::TagBased);
  }*/

  /*if (xboxController3.GetAButtonPressed())
    swerveAutoController.BeginDriveToNote();
  if (xboxController3.GetAButton())
  { 
    swerveAutoController.TurnToNote();
  }*/

  /*
  if (xboxController3.GetBButtonPressed())
      swerveAutoController.BeginDriveToNote();
  if (xboxController3.GetBButton())
  {    
    bool done = notecontroller.IntakeNoteToSelector();
    if (!done)
    {
      wristSetPoint = Intake::LOW;
      swerveAutoController.DriveToNote();
    }
  }*/

  if(xboxController3.GetBackButtonPressed()){
    flywheel.SpinFlywheelPercent(0);
  }
  else if (xboxController3.GetStartButtonPressed()){
    flywheel.SetFlywheelVelocity(SmartDashboard::GetNumber("Flywheel Setpoint", 0));
  }

  /*if (xboxController3.GetPOV() == 0){
    flywheel.PIDAngler(SmartDashboard::GetNumber("Angler Setpoint", M_PI / 2));
  }
  else{
    flywheel.MoveAnglerPercent(0);
  }*/

  /*                                                                
  ,------.         ,--.                         ,--.                
  |  .-.  \  ,---. |  |-. ,--.,--. ,---.  ,---. `--',--,--,  ,---.  
  |  |  \  :| .-. :| .-. '|  ||  || .-. || .-. |,--.|      \| .-. | 
  |  '--'  /\   --.| `-' |'  ''  '' '-' '' '-' '|  ||  ||  |' '-' ' 
  `-------'  `----' `---'  `----' .`-  / .`-  / `--'`--''--'.`-  /  
                                  `---'  `---'              `---'   
  */

  SmartDashboard::PutNumber("FL Module Heading", swerveDrive.FLModule.GetMagEncoderValue());
  SmartDashboard::PutNumber("FR Module Heading", swerveDrive.FRModule.GetMagEncoderValue());
  SmartDashboard::PutNumber("BL Module Heading", swerveDrive.BLModule.GetMagEncoderValue());
  SmartDashboard::PutNumber("BR Module Heading", swerveDrive.BRModule.GetMagEncoderValue());

  SmartDashboard::PutNumber("Odometry X Position", swerveDrive.GetOdometryPose().X().value());
  SmartDashboard::PutNumber("Odometry Y Position", swerveDrive.GetOdometryPose().Y().value());
  SmartDashboard::PutNumber("Odometry Heading", swerveDrive.GetOdometryPose().Rotation().Degrees().value());
  
  SmartDashboard::PutBoolean("Tag in View", swerveDrive.TagInView());
  SmartDashboard::PutNumber("Tag Odometry X", swerveDrive.GetTagOdometryPose().X().value());
  SmartDashboard::PutNumber("Tag Odometry Y", swerveDrive.GetTagOdometryPose().Y().value());
  SmartDashboard::PutNumber("Tag Odometry Heading", swerveDrive.GetTagOdometryPose().Rotation().Degrees().value());

  SmartDashboard::PutBoolean("Note in View", swerveDrive.NoteInView());
  SmartDashboard::PutNumber("Note Odometry X", swerveDrive.GetNoteOdometryPose().X().value());
  SmartDashboard::PutNumber("Note Odometry Y", swerveDrive.GetNoteOdometryPose().Y().value());
  SmartDashboard::PutNumber("Note Odometry Heading", swerveDrive.GetNoteOdometryPose().Rotation().Degrees().value());

  SmartDashboard::PutNumber("Elevator Encoder", ampmech.GetWinchEncoderReading());
  SmartDashboard::PutNumber("Wrist Encoder", overbumper.GetWristEncoderReading());
  SmartDashboard::PutNumber("Flywheel Encoder", flywheel.GetAnglerEncoderReading());

  SmartDashboard::PutBoolean("in intake", overbumper.GetObjectInIntake());
  SmartDashboard::PutBoolean("in mech", ampmech.GetObjectInMech());

  SmartDashboard::PutNumber("Top FlyWheel RPM", flywheel.TopFlywheel.GetMeasurement()*60.0);
  SmartDashboard::PutNumber("Bottom FlyWheel RPM", flywheel.BottomFlywheel.GetMeasurement()*60.0);

  SmartDashboard::PutNumber("Driver Mode", currentDriverMode);

  SmartDashboard::PutNumber("Climb r pos", hang.rightEncoder.GetPosition());
  SmartDashboard::PutNumber("Climb l pos", hang.leftEncoder.GetPosition());

  SmartDashboard::PutNumber("angler v", flywheel.FlywheelAnglingMotor.Get());

  SmartDashboard::PutNumber("angler a", flywheel.FlywheelAnglingMotor.GetOutputCurrent());
  SmartDashboard::PutNumber("angler temp", flywheel.FlywheelAnglingMotor.GetMotorTemperature());

  SmartDashboard::PutNumber("Top Flywheel Amperage", flywheel.TopFlywheel.m_flywheelMotor->GetOutputCurrent());
  SmartDashboard::PutNumber("Bottom Flywheel Amperage", flywheel.BottomFlywheel.m_flywheelMotor->GetOutputCurrent());
  SmartDashboard::PutNumber("Angler Amperage", flywheel.FlywheelAnglingMotor.GetOutputCurrent());
  SmartDashboard::PutNumber("Wirst Amperage", overbumper.wristMotor.GetOutputCurrent());
  SmartDashboard::PutNumber("Elevator Amperage", ampmech.winchMotor.GetSupplyCurrent().GetValue().value());
  SmartDashboard::PutNumber("FL Drive Amperage", swerveDrive.FLModule.driveMotor.GetSupplyCurrent().GetValue().value());
  //SmartDashboard::PutBoolean("climb l stop", hang.leftStop.Get());
  //SmartDashboard::PutNumber("climb l pos", hang.leftEncoder.GetPosition());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  swerveDrive.Update();

  //if(DriverStation::IsEStopped()){ 
  if(swerveDrive.TagInView()){
    lights.SetEstopped();
  }
  else if(DriverStation::IsDSAttached()){
    lights.SetIdle();
  }
  else{
    lights.SetStopped();
  }
}

void Robot::TestInit() {
    swerveDrive.ResetOdometry(Pose2d(0_m, 0_m, Rotation2d(180_deg)));
    swerveDrive.ResetTagOdometry(Pose2d(0_m, 0_m, Rotation2d(180_deg)));
    ampmech.ResetElevatorEncoder();  
    flywheel.FlywheelAnglerPID.SetupConstantTuning("Angler");
    overbumper.m_WristPID.SetupConstantTuning("Intake");
    swerveAutoController.xPIDController.SetupConstantTuning("DTP X");
    swerveAutoController.yPIDController.SetupConstantTuning("DTP Y");
    swerveAutoController.rotationPIDController.SetupConstantTuning("DTP Rot");
    swerveAutoController.noteXPIDController.SetupConstantTuning("NOTE X");
    swerveAutoController.noteYPIDController.SetupConstantTuning("NOTE Y");
    swerveAutoController.noteRotationPIDController.SetupConstantTuning("NOTE Rot");

    flywheel.SpinFlywheelPercent(0);

    SmartDashboard::PutNumber("Elevator Max Velocity", 0.7);
    SmartDashboard::PutNumber("Elevator Max Acceleration", 0.35);
    SmartDashboard::PutNumber("Elevator kP", 10);
    SmartDashboard::PutNumber("Elevator kI", 0.05);
    SmartDashboard::PutNumber("Angler Setpoint", 0.8);
    SmartDashboard::PutNumber("Flywheel Setpoint", 3000);
    SmartDashboard::PutNumber("Elevator kG", 0.5);
    SmartDashboard::PutNumber("Max Drive Speed", 0.4);
    SmartDashboard::PutNumber("Max Spin Speed", 0.4);
}

void Robot::TestPeriodic() {
  flywheel.FlywheelAnglerPID.UpdateConstantTuning("Angler");
  overbumper.m_WristPID.UpdateConstantTuning("Intake");
  ampmech.m_controller.SetP(SmartDashboard::GetNumber("Elevator kP", 10));
  ampmech.m_controller.SetI(SmartDashboard::GetNumber("Elevator kI", 0.05));
  ampmech.m_controller.SetConstraints(frc::ProfiledPIDController<units::length::meters>::Constraints{units::meters_per_second_t{SmartDashboard::GetNumber("Elevator Max Velocity", 0.7)}, units::meters_per_second_squared_t{SmartDashboard::GetNumber("Elevator Max Acceleration", 0.35)}});
  swerveAutoController.xPIDController.UpdateConstantTuning("DTP X");
  swerveAutoController.yPIDController.UpdateConstantTuning("DTP Y");
  swerveAutoController.rotationPIDController.UpdateConstantTuning("DTP Rot");
  swerveAutoController.noteXPIDController.UpdateConstantTuning("NOTE X");
  swerveAutoController.noteYPIDController.UpdateConstantTuning("NOTE Y");
  swerveAutoController.noteRotationPIDController.UpdateConstantTuning("NOTE Rot");
  swerveDrive.Update();


// Find controller input (*-1 converts values to fwd/left/counterclockwise positive)
  double leftJoystickX, leftJoystickY, rightJoystickX, rightJoystickY;
  leftJoystickY = xboxController.GetLeftY();
  leftJoystickX = xboxController.GetLeftX();
  if (allianceColor == AllianceColor::BLUE)
  {
    leftJoystickY *= -1;
    leftJoystickX *= -1;
  }

  rightJoystickX = xboxController.GetRightX() * -1;
  rightJoystickY = xboxController.GetRightY() * -1;

  // Remove ghost movement by making sure joystick is moved a certain amount
  double leftJoystickDistance = sqrt(pow(leftJoystickX, 2.0) + pow(leftJoystickY, 2.0));
  double rightJoystickDistance = sqrt(pow(rightJoystickX, 2.0) + pow(rightJoystickY, 2.0));

  if (leftJoystickDistance < CONTROLLER_DEADBAND)
  {
    leftJoystickX = 0;
    leftJoystickY = 0;
  }

  if (abs(rightJoystickDistance) < CONTROLLER_DEADBAND)
  {
    rightJoystickX = 0;
    rightJoystickY = 0;
  }

  // Find controller input (*-1 converts values to fwd/left/counterclockwise positive)
  double controller2LeftJoystickX, controller2LeftJoystickY, controller2RightJoystickX, controller2RightJoystickY;
  controller2LeftJoystickY = xboxController2.GetLeftY() * -1;
  controller2LeftJoystickX = xboxController2.GetLeftX() * -1;
  controller2RightJoystickX = xboxController2.GetRightX() * -1;
  controller2RightJoystickY = xboxController2.GetRightY();

  // Remove ghost movement by making sure joystick is moved a certain amount
  double controller2leftJoystickDistance = sqrt(pow(controller2LeftJoystickX, 2.0) + pow(controller2LeftJoystickY, 2.0));
  double controller2rightJoystickDistance = sqrt(pow(controller2RightJoystickX, 2.0) + pow(controller2RightJoystickY, 2.0));

  if (controller2leftJoystickDistance < CONTROLLER_DEADBAND)
  {
    controller2LeftJoystickX = 0;
    controller2LeftJoystickY = 0;
  }

  if (abs(controller2rightJoystickDistance) < CONTROLLER_DEADBAND)
  {
    controller2RightJoystickX = 0;
    controller2RightJoystickY = 0;
  }

  double fwdDriveSpeed = leftJoystickY * SmartDashboard::GetNumber("Max Drive Speed", 0.4);
  double strafeDriveSpeed = leftJoystickX * SmartDashboard::GetNumber("Max Drive Speed", 0.4);
  double turnSpeed = rightJoystickX * SmartDashboard::GetNumber("Max Spin Speed", 0.4);


  if(xboxController.GetBackButtonPressed()){
    flywheel.SpinFlywheelPercent(0);
  }
  else if (xboxController.GetStartButtonPressed()){
      SmartDashboard::PutBoolean("Flywheel PID Done", flywheel.SetFlywheelVelocity(SmartDashboard::GetNumber("Flywheel Setpoint", 0)));
  }

  if (xboxController.GetPOV() == 0){
    flywheel.PIDAngler(SmartDashboard::GetNumber("Angler Setpoint", M_PI / 2));
  }
  else{
    flywheel.MoveAnglerPercent(0);
  }

  if (xboxController.GetAButtonPressed())
    overbumper.BeginShootNote();

  if (xboxController.GetAButton())
    overbumper.ShootNote();
  else if (xboxController.GetBButton())
    notecontroller.IntakeNoteToSelector();
  else
    overbumper.SetIntakeMotorSpeed(0);
  
  if (xboxController.GetXButtonPressed())
    swerveAutoController.BeginDriveToNote();
  if (xboxController.GetXButton())
  {
    swerveAutoController.DriveToNote();
    bool done = notecontroller.IntakeNoteToSelector();
    if (!done)
      overbumper.PIDWristToPoint(Intake::WristSetting::LOW);
    else
      overbumper.PIDWristToPoint(Intake::WristSetting::HIGH);
  }
  else
  {
    swerveDrive.DriveSwervePercent(fwdDriveSpeed, strafeDriveSpeed, turnSpeed);
  }

  overbumper.SetIntakeMotorSpeed(0);

  /*if (xboxController.GetAButtonPressed())
    swerveAutoController.BeginDriveToPose(PoseEstimationType::PureOdometry);

  if (xboxController.GetAButton())
    swerveAutoController.DriveToPose(Pose2d(0_m, 0_m, Rotation2d(0_deg)), PoseEstimationType::PureOdometry);
  else  */
  /*if (xboxController.GetAButton()){
    overbumper.PIDWristToPoint(Intake::WristSetting::HIGH);
  }
  else if (xboxController.GetBButton()){
    overbumper.PIDWristToPoint(Intake::WristSetting::LOW);
  }
  else if (xboxController.GetXButton()){
    overbumper.PIDWristToPoint(Intake::WristSetting::SHOOT);
  }
  else if (xboxController.GetYButton()){
    overbumper.PIDWrist(overbumper.GetWristEncoderReading());
  }
  else {
    overbumper.MoveWristPercent(0);
  }*/

  if (xboxController2.GetAButtonPressed() || xboxController2.GetBButtonPressed() || xboxController2.GetYButtonPressed())
    ampmech.BeginPIDElevator();

  if (xboxController2.GetAButton()){
    ampmech.MoveToHeight(Elevator::ElevatorSetting::LOW);
  }
  else if (xboxController2.GetXButton()){
    ampmech.MoveToHeight(Elevator::ElevatorSetting::AMP);
  }
  else if (xboxController2.GetYButton()){
    ampmech.MoveToHeight(Elevator::ElevatorSetting::TRAP);
  }
  else if (xboxController2.GetBButton()){
    ampmech.winchMotor.SetVoltage(units::volt_t{SmartDashboard::GetNumber("Elevator kG", 0.5)});
  }
  else {
    ampmech.StopElevator();
  }


  SmartDashboard::PutNumber("FL Module Heading", swerveDrive.FLModule.GetMagEncoderValue());
  SmartDashboard::PutNumber("FR Module Heading", swerveDrive.FRModule.GetMagEncoderValue());
  SmartDashboard::PutNumber("BL Module Heading", swerveDrive.BLModule.GetMagEncoderValue());
  SmartDashboard::PutNumber("BR Module Heading", swerveDrive.BRModule.GetMagEncoderValue());
  SmartDashboard::PutNumber("Flywheel Magencoder Heading", flywheel.magEncoder.GetAbsolutePosition());

  SmartDashboard::PutNumber("Odometry X Position", swerveDrive.GetOdometryPose().X().value());
  SmartDashboard::PutNumber("Odometry Y Position", swerveDrive.GetOdometryPose().Y().value());
  SmartDashboard::PutNumber("Odometry Heading", swerveDrive.GetOdometryPose().Rotation().Degrees().value());
  
  SmartDashboard::PutBoolean("Tag in View", swerveDrive.TagInView());
  SmartDashboard::PutNumber("Tag Odometry X", swerveDrive.GetTagOdometryPose().X().value());
  SmartDashboard::PutNumber("Tag Odometry Y", swerveDrive.GetTagOdometryPose().Y().value());
  SmartDashboard::PutNumber("Tag Odometry Heading", swerveDrive.GetTagOdometryPose().Rotation().Degrees().value());

  SmartDashboard::PutBoolean("Note in View", swerveDrive.NoteInView());
  SmartDashboard::PutNumber("Note Odometry X", swerveDrive.GetNoteOdometryPose().X().value());
  SmartDashboard::PutNumber("Note Odometry Y", swerveDrive.GetNoteOdometryPose().Y().value());
  SmartDashboard::PutNumber("Note Odometry Heading", swerveDrive.GetNoteOdometryPose().Rotation().Degrees().value());

  SmartDashboard::PutNumber("Elevator Encoder", ampmech.GetWinchEncoderReading());
  SmartDashboard::PutNumber("Wrist Encoder", overbumper.GetWristEncoderReading());
  SmartDashboard::PutNumber("Flywheel Encoder", flywheel.GetAnglerEncoderReading());

  SmartDashboard::PutNumber("Top FlyWheel RPM", flywheel.TopFlywheel.GetMeasurement()*60.0);
  SmartDashboard::PutNumber("Bottom FlyWheel RPM", flywheel.BottomFlywheel.GetMeasurement()*60.0);
  SmartDashboard::PutBoolean("Top FlyWheel Done", flywheel.TopFlywheel.AtSetpoint());
  SmartDashboard::PutBoolean("Bottom FlyWheel Done", flywheel.BottomFlywheel.AtSetpoint());

  SmartDashboard::PutBoolean("in intake", overbumper.GetObjectInIntake());
  SmartDashboard::PutBoolean("in mech", ampmech.GetObjectInMech());
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
