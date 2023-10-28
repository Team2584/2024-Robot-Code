#pragma once

#include "Robot.h"

using namespace std;
using namespace frc;

XboxController *xbox_Drive = new XboxController(0);
XboxController *xbox_Drive2 = new XboxController(1);

rev::CANSparkMax swerveFL{11, rev::CANSparkMax::MotorType::kBrushless};
ctre::phoenix6::hardware::TalonFX driveFL{01};
rev::CANSparkMax swerveFR{12, rev::CANSparkMax::MotorType::kBrushless};
ctre::phoenix6::hardware::TalonFX driveFR{02};
rev::CANSparkMax swerveBL{13, rev::CANSparkMax::MotorType::kBrushless};
ctre::phoenix6::hardware::TalonFX driveBL{03};
rev::CANSparkMax swerveBR{14, rev::CANSparkMax::MotorType::kBrushless};
ctre::phoenix6::hardware::TalonFX driveBR{04};

DutyCycleEncoder FLMagEnc(8);
DutyCycleEncoder FRMagEnc(6);
DutyCycleEncoder BLMagEnc(9);
DutyCycleEncoder BRMagEnc(7);

ctre::phoenix6::hardware::Pigeon2 _pigeon(6);

