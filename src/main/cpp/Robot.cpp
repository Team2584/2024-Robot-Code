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
#include "LEDs.h"
#include "NoteController.h"

VisionSwerve swerveDrive{};
XboxController xboxController{0};
XboxController xboxController2{1};
XboxController xboxController3{2};
Intake overbumper{};
FlywheelSystem flywheel{};
Elevator ampmech{};
Climb hang{&swerveDrive};
NoteController notecontroller{&overbumper, &flywheel, &ampmech};
LEDLights lightStrip{0};

SwerveDriveAutonomousController swerveAutoController{&swerveDrive};
AutonomousShootingController flywheelController{&swerveAutoController, &flywheel, &overbumper, &ampmech};
AutonomousAmpingController autoAmpController{&swerveAutoController, &notecontroller};
AutonomousTrapController autoTrapController{&notecontroller, &ampmech, &hang};

AutonomousController autoController{&swerveDrive, &overbumper, &flywheel, &ampmech, &swerveAutoController, &notecontroller, &flywheelController, &autoAmpController};

Elevator::ElevatorSetting elevSetHeight = Elevator::LOW;
Intake::WristSetting wristSetPoint = Intake::HIGH;
bool anglingToSpeaker = false;

AllianceColor allianceColor = AllianceColor::BLUE;

enum DRIVER_MODE {BASIC, AUTO_AIM_STATIONARY, SHOOT_ON_THE_MOVE, AUTO_AMP, CLIMBING_TRAP};
DRIVER_MODE currentDriverMode = DRIVER_MODE::BASIC;

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoBCSI2S, kAutoBCSI2S);
  m_chooser.AddOption(kAutoBLSI3S, kAutoBLSI3S);
  m_chooser.AddOption(kAutoBRSI1S, kAutoBRSI1S);
  m_chooser.AddOption(kAutoRCSI10S, kAutoRCSI10S);
  m_chooser.AddOption(kAutoRLSI9S, kAutoRLSI9S);
  m_chooser.AddOption(kAutoRRSI11S, kAutoRRSI11S);
  m_chooser.AddOption(kAutoBR4CloseNotes, kAutoBR4CloseNotes);
  m_chooser.AddOption(kAutoBLSI3SI8S, kAutoBLSI3SI8S);
  m_chooser.AddOption(kAutoBLSS3S8TEST, kAutoBLSS3S8TEST);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  
  SmartDashboard::PutNumber("Flywheel Setpoint", 0);
  SmartDashboard::PutNumber("Angler Setpoint", M_PI / 2);
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
  swerveDrive.UpdateRaspiConnection();

  if (DriverStation::GetMatchType() != DriverStation::MatchType::kNone)
  {
    if (DriverStation::GetAlliance() == DriverStation::kRed)
      allianceColor = AllianceColor::RED;
    else
      allianceColor = AllianceColor::BLUE;
  }
  
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

  if (m_autoSelected == kAutoBCSI2S)
  {
    autoController.SetupBlueCenterShootIntake2Shoot(); 
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoBLSI3S)
  {
    autoController.SetupBlueLeftShootIntake3Shoot();
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoBRSI1S)
  {
    autoController.SetupBlueRightShootIntake1Shoot();
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoRLSI9S)
  {
    autoController.SetupRedLeftShootIntake9Shoot();
    allianceColor = AllianceColor::RED;
  }
  else if (m_autoSelected == kAutoRCSI10S)
  {
    autoController.SetupRedCenterShootIntake10Shoot();
    allianceColor = AllianceColor::RED;
  }
  else if (m_autoSelected == kAutoRRSI11S)
  {
    autoController.SetupRedRightShootIntake11Shoot();
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
  else if (m_autoSelected == kAutoBR4CloseNotes)
  {
    autoController.SetupBlueRightShootIntake1ShootIntake2ShootIntake3();
    allianceColor = AllianceColor::BLUE;
  }
  else if (m_autoSelected == kAutoFASTTEST)
  {
    autoController.SetupFastBlueRightShootIntake1ShootIntake2ShootIntake3();
    allianceColor = AllianceColor::BLUE;  
  }
}

void Robot::AutonomousPeriodic()
{
  /* Updates */
  swerveDrive.Update();

   /* Autos */
  if (m_autoSelected == kAutoBCSI2S)
    autoController.BlueCenterShootIntake2Shoot();
  else if (m_autoSelected == kAutoBLSI3S)
    autoController.BlueLeftShootIntake3Shoot();
  else if (m_autoSelected == kAutoBRSI1S)
    autoController.BlueRightShootIntake1Shoot();
  else if (m_autoSelected == kAutoRCSI10S)
    autoController.RedCenterShootIntake10Shoot();
  else if (m_autoSelected == kAutoRLSI9S)
    autoController.RedLeftShootIntake9Shoot();
  else if (m_autoSelected == kAutoRRSI11S)
    autoController.RedRightShootIntake11Shoot();
  else if (m_autoSelected == kAutoBLSI3SI8S)
    autoController.BlueLeftShootIntake3ShootIntake8Shoot();
  else if (m_autoSelected == kAutoBLSS3S8TEST)
    autoController.BlueLeftShootIntake3ShootIntake8ShootTESTING(); 
  else if (m_autoSelected == kAutoBR4CloseNotes)
    autoController.BlueRightShootIntake1ShootIntake2ShootIntake3();
  else if (m_autoSelected == kAutoFASTTEST)
    autoController.FastBlueRightShootIntake1ShootIntake2ShootIntake3();
}

void Robot::TeleopInit()
{
  swerveDrive.ResetOdometry(Pose2d(0.74_m, 4.35_m, Rotation2d(120_deg)));
  swerveDrive.ResetTagOdometry(Pose2d(0_m, 0_m, Rotation2d(180_deg)));
  ampmech.ResetElevatorEncoder();  
  currentDriverMode = DRIVER_MODE::BASIC;
  
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
      if (xboxController.GetLeftBumper())
      {
        fwdDriveSpeed = leftJoystickY * SPEED_BOOST_DRIVE;
        strafeDriveSpeed = leftJoystickX * SPEED_BOOST_DRIVE;
        turnSpeed = rightJoystickX * SPEED_BOOST_SPIN;      
      }

      swerveDrive.DriveSwervePercent(fwdDriveSpeed, strafeDriveSpeed, turnSpeed);

      // These function calls "prepare" the true function calls below
      if (xboxController2.GetRightBumperPressed()){
        notecontroller.BeginFromElevatorToSelector();
      }
      if (xboxController2.GetBButtonPressed()){
        notecontroller.BeginScoreNoteInPosition(Elevator::ElevatorSetting::AMP);
      }

      // Note Controller Stuff
      wristSetPoint = Intake::SHOOT;

      if (xboxController.GetRightBumper())
      {
        bool done = notecontroller.IntakeNoteToSelector();
        if (!done)
          wristSetPoint = Intake::LOW;
      }
      else if (xboxController.GetRightTriggerAxis() > 0.5)
      {
        overbumper.OuttakeNote();
      }
      else if(xboxController2.GetRightTriggerAxis() > 0.5){
        notecontroller.ToElevator();
      }
      else if(xboxController2.GetLeftTriggerAxis() > 0.5){
        overbumper.ShootNote();
      }
      else if (xboxController2.GetRightBumper()){
        notecontroller.FromElevatorToSelector();
      }
      else if (xboxController2.GetLeftBumper()){
        ampmech.NoteFromSelector();
      }
      else if (xboxController2.GetAButton())
      {
        notecontroller.LiftNoteToPosition(Elevator::ElevatorSetting::AMP);
      }
      else if (xboxController2.GetBButton())
      {
        notecontroller.ScoreNoteInPosition(Elevator::ElevatorSetting::AMP);
      }
      else {
        overbumper.SetIntakeMotorSpeed(0);
        ampmech.SetAmpMotorPercent(0);
        ampmech.StopElevator();
      }

      overbumper.PIDWristToPoint(wristSetPoint);

      // Keep flywheel ready for close shots
      //flywheel.SetFlywheelVelocity(3000);
      flywheel.SpinFlywheelPercent(0);
      flywheel.PIDAngler(0.8); // change this to clear chain

      if(xboxController.GetStartButtonPressed()){
        hang.climbZeroed = false;
      }

      // Switching Driver Mode
      if (xboxController2.GetXButtonPressed())
      {
        flywheelController.BeginAimAndFire(allianceColor);
        currentDriverMode = DRIVER_MODE::AUTO_AIM_STATIONARY;
      }
      if (xboxController2.GetStartButtonPressed()){
        autoAmpController.BeginDriveToAmp(allianceColor);
        currentDriverMode = DRIVER_MODE::AUTO_AMP;
      }
      if (xboxController2.GetYButtonPressed())
      {
        notecontroller.BeginScoreNoteInPosition(Elevator::ElevatorSetting::AMP);
        flywheelController.BeginAimAndFire(allianceColor);
        currentDriverMode = DRIVER_MODE::SHOOT_ON_THE_MOVE;
      }

      if(xboxController2.GetAButton() || xboxController.GetPOV() == 180 || xboxController.GetBButton() || xboxController.GetXButton() || xboxController.GetBackButton()){
        currentDriverMode = DRIVER_MODE::CLIMBING_TRAP;
      }

      break;
    }

    case DRIVER_MODE::AUTO_AIM_STATIONARY:
    {
      bool doneShooting = flywheelController.AimAndFire(allianceColor);
    
      overbumper.PIDWristToPoint(Intake::SHOOT);

      if (!xboxController2.GetXButton() || doneShooting)
        currentDriverMode = DRIVER_MODE::BASIC;
    
      break;
    }

    case DRIVER_MODE::SHOOT_ON_THE_MOVE:
    {
      double fwdDriveSpeed = leftJoystickY * MAX_DRIVE_SPEED_SHOOT_ON_THE_MOVE;
      double strafeDriveSpeed = leftJoystickX * MAX_DRIVE_SPEED_SHOOT_ON_THE_MOVE;

      flywheelController.TurnToSpeakerWhileDriving(fwdDriveSpeed, strafeDriveSpeed, allianceColor);
      flywheelController.SpinFlywheelForSpeaker(allianceColor);
      flywheelController.AngleFlywheelToSpeaker(allianceColor);
      flywheelController.ClearElevatorForShot();

      if(xboxController2.GetLeftTriggerAxis() > 0.5)
        overbumper.ShootNote();
      else
        overbumper.SetIntakeMotorSpeed(0);

      if (!xboxController2.GetYButton())
        currentDriverMode = DRIVER_MODE::BASIC;

      break;
    }

    case DRIVER_MODE::AUTO_AMP:
    {
      bool isAtAmp = autoAmpController.DriveToAmp(allianceColor);
      bool scored = false;

      if(isAtAmp){
        scored = notecontroller.ScoreNoteInPosition(Elevator::ElevatorSetting::AMP);
      }

      if(!xboxController2.GetStartButton() || scored){
        currentDriverMode = DRIVER_MODE::BASIC;
      }

      break;
    }

    case DRIVER_MODE::CLIMBING_TRAP:
    {
      double fwdDriveSpeed = leftJoystickY * MAX_DRIVE_SPEED_CLIMB;
      double strafeDriveSpeed = leftJoystickX * MAX_DRIVE_SPEED_CLIMB;
      double turnSpeed = rightJoystickX * MAX_SPIN_SPEED_CLIMB;

      swerveDrive.DriveSwervePercent(fwdDriveSpeed, strafeDriveSpeed, turnSpeed);

      flywheel.PIDAngler(M_PI/2);

      if(xboxController.GetAButton()){
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
      else if(xboxController.GetXButton()){
        autoTrapController.PrepareClimb();
      }
      else if (xboxController.GetYButton()){
         if(autoTrapController.ClimbToTrap()){
            notecontroller.ScoreNoteInPosition(Elevator::ElevatorSetting::TRAP);
         }
      }
      else{
        hang.HoldClimb();
      }

      if (!xboxController2.GetAButton() && !(xboxController.GetPOV() != -1) && !xboxController.GetBButton() && !xboxController.GetXButton() && !xboxController.GetBackButton())
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
    swerveAutoController.LoadTrajectory("1To2");
    swerveAutoController.BeginNextTrajectory();
  }
  if (xboxController3.GetBButton())
  {
    swerveAutoController.FollowTrajectory(PoseEstimationType::TagBased);
  }

  // Follow spline for testing
  if (xboxController3.GetAButtonPressed())
  {
    swerveAutoController.ResetTrajectoryQueue();
    swerveAutoController.LoadTrajectory("Test");
    swerveAutoController.BeginNextTrajectory();
  }
  if (xboxController3.GetAButton())
  {
    swerveAutoController.FollowTrajectory(PoseEstimationType::TagBased);
  }

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
  
  SmartDashboard::PutNumber("FL Drive Encoder", swerveDrive.FLModule.GetDriveEncoder());
    SmartDashboard::PutNumber("FR Drive Encoder", swerveDrive.FRModule.GetDriveEncoder());
  SmartDashboard::PutNumber("BL Drive Encoder", swerveDrive.BLModule.GetDriveEncoder());

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
  SmartDashboard::PutBoolean("in tunnel", overbumper.GetObjectInTunnel());

  SmartDashboard::PutNumber("Top FlyWheel RPM", flywheel.TopFlywheel.GetMeasurement()*60.0);
  SmartDashboard::PutNumber("Bottom FlyWheel RPM", flywheel.BottomFlywheel.GetMeasurement()*60.0);

  SmartDashboard::PutNumber("Driver Mode", currentDriverMode);

  SmartDashboard::PutNumber("Climb r pos", hang.rightEncoder.GetPosition());
  SmartDashboard::PutNumber("Climb l pos", hang.leftEncoder.GetPosition());

  //SmartDashboard::PutBoolean("climb l stop", hang.leftStop.Get());
  //SmartDashboard::PutNumber("climb l pos", hang.leftEncoder.GetPosition());
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
