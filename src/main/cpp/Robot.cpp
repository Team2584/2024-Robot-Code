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
#include "Autonomous Functionality/AutonomousRoutines.h"
#include "Intake.h"
#include "FlyWheel.h"
#include "Elevator.h"
#include "Climb.h"
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

SwerveDriveAutonomousController swerveAutoController{&swerveDrive};
AutonomousShootingController flywheelController{&swerveAutoController, &flywheel};
AutonomousAmpingController autoAmpController{&swerveAutoController, &notecontroller};

AutonomousController autoController{&swerveAutoController, &notecontroller, &flywheelController, &autoAmpController};

Elevator::ElevatorSetting elevSetHeight = Elevator::LOW;
Intake::WristSetting wristSetPoint = Intake::HIGH;
bool anglingToSpeaker = false;


void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoBCSI2S, kAutoBCSI2S);
  m_chooser.AddOption(kAutoBLSI3S, kAutoBLSI3S);
  m_chooser.AddOption(kAutoBRSI1S, kAutoBRSI1S);
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

  if (m_autoSelected == kAutoBCSI2S)
    autoController.SetupBlueCenterShootIntake2Shoot(); 
  else
  {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic()
{
  if (m_autoSelected == kAutoBCSI2S)
    autoController.BlueCenterShootIntake2Shoot();
  else
  {
    // Default Auto goes here
  }
}

void Robot::TeleopInit()
{
  swerveDrive.ResetOdometry(Pose2d(0_m, 0_m, Rotation2d(180_deg)));
  swerveDrive.ResetTagOdometry(Pose2d(0_m, 0_m, Rotation2d(180_deg)));
  ampmech.ResetElevatorEncoder();  
}

void Robot::TeleopPeriodic()
{
  /* UPDATES */

  swerveDrive.Update();

  /* Controller Data */

  // Find controller input (*-1 converts values to fwd/left/counterclockwise positive)
  double leftJoystickX, leftJoystickY, rightJoystickX, rightJoystickY;
  leftJoystickY = xboxController.GetLeftY() * -1;
  leftJoystickX = xboxController.GetLeftX() * -1;
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

  // Scale control values to max speed
  double fwdDriveSpeed = leftJoystickY * MAX_DRIVE_SPEED;
  double strafeDriveSpeed = leftJoystickX * MAX_DRIVE_SPEED;
  double turnSpeed = rightJoystickX * MAX_SPIN_SPEED;

  SmartDashboard::PutNumber("FWD Drive", fwdDriveSpeed);
  SmartDashboard::PutNumber("Strafe Drive", strafeDriveSpeed);
  SmartDashboard::PutNumber("Turn Drive", turnSpeed);


  // Drive the robot
  swerveDrive.DriveSwervePercent(strafeDriveSpeed, fwdDriveSpeed, turnSpeed);

  if (xboxController3.GetAButton())
  { 
    swerveAutoController.TurnToNote();
  }

  // Drive to a position for testing
  if (xboxController3.GetRightBumperPressed())
    swerveAutoController.BeginDriveToPose(PoseEstimationType::PureOdometry);
  if (xboxController3.GetRightBumper())
    swerveAutoController.DriveToPose(Pose2d(-0.5_m,0_m,Rotation2d(90_deg)), PoseEstimationType::PureOdometry);

  if (xboxController3.GetLeftBumperPressed())
    swerveAutoController.BeginDriveToPose(PoseEstimationType::TagBased);
  if (xboxController3.GetLeftBumper())
    swerveAutoController.DriveToPose(ElevatorConstants::AMP_SCORING_POSITION, PoseEstimationType::TagBased);

  if (xboxController3.GetYButtonPressed())
    autoAmpController.BeginDriveToAmp();
  else
    autoAmpController.DriveToAmp();
    
  /*
  // Follow spline for testing
  if (xboxController.GetBButtonPressed())
  {
    swerveAutoController.ResetTrajectoryQueue();
    swerveAutoController.LoadTrajectory("Test");
    swerveAutoController.BeginNextTrajectory();
  }
  if (xboxController.GetBButton())
  {
    swerveAutoController.FollowTrajectory(PoseEstimationType::PureOdometry);
  }*/

  /*                                            
   _  _     _          ___         _           _ _         
  | \| |___| |_ ___   / __|___ _ _| |_ _ _ ___| | |___ _ _ 
  | .` / _ \  _/ -_) | (__/ _ \ ' \  _| '_/ _ \ | / -_) '_|
  |_|\_\___/\__\___|  \___\___/_||_\__|_| \___/_|_\___|_|                                                     
  */

  wristSetPoint = Intake::SHOOT;

  // These function calls "prepare" the true function calls below
  if (xboxController2.GetRightBumperPressed()){
    notecontroller.BeginFromElevatorToSelector();
  }
  if (xboxController2.GetBButtonPressed()){
    notecontroller.BeginScoreNoteInPosition(Elevator::ElevatorSetting::AMP);
  }
  if (xboxController2.GetYButtonPressed()){
    notecontroller.BeginScoreNoteInPosition(Elevator::ElevatorSetting::TRAP);
  }

  // This is the actual control scheme
  if (xboxController.GetRightBumper())
  {
    bool done = notecontroller.IntakeNoteToSelector();
    if (!done)
      wristSetPoint = Intake::LOW;
  }
  else if (xboxController.GetLeftBumper())
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
  else if (xboxController2.GetXButton())
  {
    notecontroller.LiftNoteToPosition(Elevator::ElevatorSetting::TRAP);
  }
  else if (xboxController2.GetAButton())
  {
    notecontroller.LiftNoteToPosition(Elevator::ElevatorSetting::AMP);
  }
  else if (xboxController2.GetBButton())
  {
    notecontroller.ScoreNoteInPosition(Elevator::ElevatorSetting::AMP);
  }
  else if (xboxController2.GetYButton())
  {
    notecontroller.ScoreNoteInPosition(Elevator::ElevatorSetting::TRAP);
  }
  else {
    overbumper.SetIntakeMotorSpeed(0);
    ampmech.SetAmpMotorPercent(0);
    ampmech.StopElevator();
  }

  
  /*                                                      
  ,------.,--.                    ,--.                   ,--. 
  |  .---'|  |,--. ,--.,--.   ,--.|  ,---.  ,---.  ,---. |  | 
  |  `--, |  | \  '  / |  |.'.|  ||  .-.  || .-. :| .-. :|  | 
  |  |`   |  |  \   '  |   .'.   ||  | |  |\   --.\   --.|  | 
  `--'    `--'.-'  /   '--'   '--'`--' `--' `----' `----'`--'                                          
  */

  if(xboxController2.GetBackButtonPressed()){
    flywheel.SpinFlywheelPercent(0);
  }
  else if (xboxController2.GetStartButtonPressed()){
    flywheel.SetFlywheelVelocity(SmartDashboard::GetNumber("Flywheel Setpoint", 0));
  }


  if (xboxController2.GetPOV() == 0){
    flywheel.PIDAngler(SmartDashboard::GetNumber("Angler Setpoint", M_PI / 2));
  }
  else{
    flywheel.MoveAnglerPercent(0);
  }


  /*if(xboxController3.GetAButtonPressed()){
    anglingToSpeaker = !anglingToSpeaker;
  }

  if (anglingToSpeaker){
    flywheelController.AngleFlywheelToSpeaker();
  }*/
  
  if (xboxController3.GetXButton()){
    flywheelController.TurnToSpeaker();
  }

  //PID Intake wrist
  overbumper.PIDWristToPoint(wristSetPoint);

  /*                                                   
  ,------.,--.                         ,--.                  /  //  /,---.                   ,--.   ,--.             ,--.      
  |  .---'|  | ,---.,--.  ,--.,--,--.,-'  '-. ,---. ,--.--. /  //  //  O  \ ,--,--,--. ,---. |   `.'   | ,---.  ,---.|  ,---.  
  |  `--, |  || .-. :\  `'  /' ,-.  |'-.  .-'| .-. ||  .--'/  //  /|  .-.  ||        || .-. ||  |'.'|  || .-. :| .--'|  .-.  | 
  |  `---.|  |\   --. \    / \ '-'  |  |  |  ' '-' '|  |  /  //  / |  | |  ||  |  |  || '-' '|  |   |  |\   --.\ `--.|  | |  | 
  `------'`--' `----'  `--'   `--`--'  `--'   `---' `--' /  //  /  `--' `--'`--`--`--'|  |-' `--'   `--' `----' `---'`--' `--'
  */

  //For testing - probably don't want to use this enum in final code and for sure not in this way
  //We will need to add an (object in elevator) check
  
  /*if (xboxController.GetBButtonPressed()){
    //This line of code cycles to the next value in a 3-value Enumerator (of elevator positions), or cycles back to the first if it's currently at the third
    elevSetHeight = static_cast<Elevator::ElevatorSetting>((elevSetHeight + 1) % 4); 
  }
  if (xboxController.GetBButton())
  {
    ampmech.MoveToHeight(elevSetHeight);
  }*/
  


  /*
   ,-----.,--.,--.           ,--.    
  '  .--./|  |`--',--,--,--.|  |-.  
  |  |    |  |,--.|        || .-. ' 
  '  '--'\|  ||  ||  |  |  || `-' | 
   `-----'`--'`--'`--`--`--' `---'  
  */

  /*if (controller2LeftJoystickY != 0 || controller2RightJoystickY != 0)
    hang.SetClimbMotors(controller2LeftJoystickY, controller2RightJoystickY);
  else if(xboxController.GetAButton()){
    hang.ExtendClimb();
  }
  else if (xboxController.GetBButton()){
    hang.RetractClimb();
  }
  else if (xboxController.GetXButton()){
    hang.ZeroClimb();
  }
  else if (xboxController.GetPOV() == 90){
    hang.ClimbPID(-0.6);
  }else if (xboxController.GetPOV() == 270){
    hang.ClimbPID(0);
  }
  else{
    hang.HoldClimb();
  }

  if(xboxController.GetYButtonPressed()){
    hang.climbZeroed = false;
  }*/

  //ONLY uncomment this when motors are found to be going the right directions and limits work properly
  /*
  if(!hang.climbZeroed){
    hang.ZeroClimb();
  }
  */

  /*if(xboxController2.GetAButton()){
    hang.ExtendClimb();
  }
  else if (xboxController2.GetBButton()){
    hang.RetractClimb();
    
  }
  else if (xboxController2.GetXButton()){
    hang.ZeroClimb();
  }
  else if (xboxController2.GetYButton()){
    hang.BalanceWhileClimbing();
  }
  else{
    hang.HoldClimb();
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

  SmartDashboard::PutNumber("Elevator Encoder", ampmech.GetWinchEncoderReading());
  SmartDashboard::PutNumber("Wrist Encoder", overbumper.GetWristEncoderReading());
  SmartDashboard::PutNumber("Flywheel Encoder", flywheel.GetAnglerEncoderReading());

  SmartDashboard::PutBoolean("in intake", overbumper.GetObjectInIntake());
  SmartDashboard::PutBoolean("in mech", ampmech.GetObjectInMech());
  SmartDashboard::PutBoolean("in tunnel", overbumper.GetObjectInTunnel());

  SmartDashboard::PutNumber("Top FlyWheel RPM", flywheel.TopFlywheel.GetMeasurement());
  SmartDashboard::PutNumber("Bottom FlyWheel RPM", flywheel.BottomFlywheel.GetMeasurement());

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
