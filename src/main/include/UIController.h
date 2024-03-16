#include "Robot.h"
#include "Elevator.h"
#include "FlyWheel.h"
#include "Climb.h"
#include "Intake.h"
#include "VisionBasedSwerve.h"
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/smartdashboard/SendableBuilderImpl.h>
#include <networktables/GenericEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>

#ifndef SMC_H
#define SMC_H

class SwerveSendable : public wpi::Sendable {
public:

    VisionSwerve * swe;
    SwerveSendable(VisionSwerve *rve): swe{rve}{}

    void InitSendable(wpi::SendableBuilder& builder) override {

        builder.SetSmartDashboardType("SwerveDrive");

        builder.AddDoubleProperty("Front Left Angle", [this] { return swe->FLModule.GetModuleHeading() ; }, nullptr);
        builder.AddDoubleProperty("Front Left Velocity", [this] { return swe->FLModule.GetDriveMotorVelocity().value(); }, nullptr);

        builder.AddDoubleProperty("Front Right Angle", [this] { return swe->FRModule.GetModuleHeading() ; }, nullptr);
        builder.AddDoubleProperty("Front Right Velocity", [this] { return swe->FRModule.GetDriveMotorVelocity().value(); }, nullptr);

        builder.AddDoubleProperty("Back Left Angle", [this] { return swe->BLModule.GetModuleHeading() ; }, nullptr);
        builder.AddDoubleProperty("Back Left Velocity", [this] { return swe->BLModule.GetDriveMotorVelocity().value(); }, nullptr);

        builder.AddDoubleProperty("Back Right Angle", [this] { return swe->BRModule.GetModuleHeading() ; }, nullptr);
        builder.AddDoubleProperty("Back Right Velocity", [this] { return swe->BRModule.GetDriveMotorVelocity().value(); }, nullptr);

        builder.AddDoubleProperty("Robot Angle", [this] { return swe->GetIMUHeading() ; }, nullptr);
    }
};

class ClimbElevatorSendable : public wpi::Sendable {
public:

    Elevator * swe;
    Climb * rve;
    ClimbElevatorSendable(Elevator *elev, Climb *clim): swe{elev},rve{clim}{}

    void InitSendable(wpi::SendableBuilder& builder) override {

        builder.SetSmartDashboardType("Elevator Climb");

        builder.AddDoubleProperty("Elevator Position", [this] { return abs(swe->GetWinchEncoderReading()) ; }, nullptr);
        builder.AddDoubleProperty("Elevator Setpoint", [this] { return abs(swe->GetElevatorSetpoint()) ; }, nullptr);
        builder.AddBooleanProperty("Elevator At Setpoint", [this] { return swe->GetElevatorAtSetpoint() ; }, nullptr);

        builder.AddDoubleProperty("Climb Left Position", [this] { return rve->leftEncoder.GetPosition() ; }, nullptr);
        builder.AddDoubleProperty("Climb Left Setpoint", [this] { return rve->leftPID.GetSetpoint().position.value() ; }, nullptr);

        builder.AddDoubleProperty("Climb Right Position", [this] { return rve->rightEncoder.GetPosition() ; }, nullptr);
        builder.AddDoubleProperty("Climb Right Setpoint", [this] { return rve->rightPID.GetSetpoint().position.value() ; }, nullptr);

        builder.AddBooleanProperty("Climb At Setpoint", [this] { return rve->GetClimbAtPos() ; }, nullptr);

    }

};

class FlywheelSendable : public wpi::Sendable {
public:

    FlywheelSystem *fly;
    FlywheelSendable(FlywheelSystem *fly_): fly{fly_}{}

    void InitSendable(wpi::SendableBuilder& builder) override {

        builder.SetSmartDashboardType("Flywheel System");

        builder.AddDoubleProperty("Angler Position", [this] { return fly->GetAnglerEncoderReading() ; }, nullptr);
        builder.AddDoubleProperty("Angler Setpoint", [this] { return fly->FlywheelAnglerPID.GetPIDSetpoint() ; }, nullptr);
        builder.AddBooleanProperty("Angler At Setpoint", [this] { return fly->FlywheelAnglerPID.PIDFinished() ; }, nullptr);

        builder.AddDoubleProperty("Bottom Flywheel Velocity", [this] { return fly->BottomFlywheel.GetMeasurement() ; }, nullptr);
        builder.AddDoubleProperty("Bottom Flywheel Setpoint", [this] { return fly->BottomFlywheel.m_shooterPID.GetSetpoint() ; }, nullptr);
        builder.AddBooleanProperty("Bottom Flywheel At Setpoint", [this] { return fly->BottomFlywheel.m_shooterPID.AtSetpoint() ; }, nullptr);

        builder.AddDoubleProperty("Top Flywheel Velocity", [this] { return fly->TopFlywheel.GetMeasurement() ; }, nullptr);
        builder.AddDoubleProperty("Top Flywheel Setpoint", [this] { return fly->TopFlywheel.m_shooterPID.GetSetpoint() ; }, nullptr);
        builder.AddBooleanProperty("Top Flywheel At Setpoint", [this] { return fly->TopFlywheel.m_shooterPID.AtSetpoint() ; }, nullptr);

    }

};

class IntakeSendable : public wpi::Sendable {
public:

    Intake *swe;
    Elevator *elev;
    IntakeSendable(Intake* rve, Elevator *elev_): swe{rve}, elev{elev_}{}

    void InitSendable(wpi::SendableBuilder& builder) override {

        builder.SetSmartDashboardType("Intake");

        builder.AddDoubleProperty("Wrist Position", [this] { return swe->GetWristEncoderReading(); }, nullptr);
        builder.AddDoubleProperty("Wrist Setpoint", [this] { return swe->m_WristPID.GetPIDSetpoint() ; }, nullptr);

        builder.AddBooleanProperty("Note In Intake", [this] { return swe->GetObjectInIntake() ; }, nullptr);

        builder.AddBooleanProperty("Note In Elevator", [this] { return elev->GetObjectInMech() ; }, nullptr);

    }

};

class SmartDashboardController {

public:

    VisionSwerve *swerveDrive; /* A reference to our swerve drive. */
    Intake *intake;
    Elevator *elevator;
    FlywheelSystem *flywheel;
    Climb *climb;

    frc::PowerDistribution *pdps;

    frc::ShuffleboardTab& autontab = Shuffleboard::GetTab("Auton");
    frc::ShuffleboardTab& teleoptab = Shuffleboard::GetTab("TeleOp");

    int currenttab = 0;
    
    frc::Field2d m_field;

    frc::ShuffleboardLayout& elevatorlist = teleoptab.GetLayout("Elevator", frc::BuiltInLayouts::kList).WithSize(2, 3).WithPosition(8,0);
    //frc::ShuffleboardLayout& flywheellist = teleoptab.GetLayout("Flywheel", frc::BuiltInLayouts::kList).WithSize(2, 4).WithPosition(6,0);

    SwerveSendable ntswervedrive_{swerveDrive};
    ClimbElevatorSendable ntclimbelevator_{elevator, climb};
    FlywheelSendable ntflywheel_{flywheel};
    IntakeSendable ntintake_{intake, elevator};

    SmartDashboardController(VisionSwerve *swerveDrive_, Intake *intake_, FlywheelSystem *flywheel_, Elevator *ampMech_, frc::PowerDistribution *pdp, Climb *climb_)
    :
        swerveDrive{swerveDrive_},
        intake{intake_},
        elevator{ampMech_},
        flywheel{flywheel_},
        climb{climb_},
        pdps{pdp}

    {

        m_field.SetRobotPose(swerveDrive->GetOdometryPose());
        teleoptab.Add("Field", m_field).WithSize(3, 2).WithPosition(3, 0).WithWidget(frc::BuiltInWidgets::kField);
        teleoptab.Add("PDP", *pdps).WithSize(3, 4).WithPosition(0, 0).WithWidget(frc::BuiltInWidgets::kPowerDistribution);  
        teleoptab.Add("Swerve Drive", ntswervedrive_).WithSize(2,2).WithPosition(3,2);


        elevatorlist.Add("Alt Intake", false).WithSize(1, 1).WithPosition(6, 0).WithWidget(frc::BuiltInWidgets::kToggleButton);
        elevatorlist.Add("Zero Elevator", false).WithSize(1, 1).WithPosition(6, 0).WithWidget(frc::BuiltInWidgets::kCommand);

        teleoptab.Add("Flywheel System", ntflywheel_).WithSize(2, 4).WithPosition(6,0);
        elevatorlist.Add("Elevator", ntclimbelevator_).WithSize(2,4).WithPosition(3,2);


    }

    void Update(){
        m_field.SetRobotPose(swerveDrive->GetTagOdometryPose());
        SmartDashboard::PutNumber("SwitchMode", currenttab);
        Shuffleboard::Update();
    }

    void SwitchTab(int tab){
        currenttab = tab;
    }

};

#endif