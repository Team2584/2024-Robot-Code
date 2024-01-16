# 2024 Flame of the West Robot Code Base

## Important Resources

- This year's main Github: https://github.com/Team2584/2024-Robot-Code
- This year's vision Github: https://github.com/Team2584/2024-vision

- Installing WPILib 2024 (Required): https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html
- Installing NI FRC Game Tools (Driver Station, Firmware Utility, Shuffleboard) (Not Required): https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html

- FIRST Documentation: https://docs.wpilib.org/en/stable/index.html
- REV Documentation: https://codedocs.revrobotics.com/cpp/index.html
- REV Robot Code Examples: https://github.com/REVrobotics/SPARK-MAX-Examples
- PhotonVision (Apriltags): https://docs.photonvision.org/en/latest/

## How to find your way around the code
- The file main/cpp/Robot.cpp contains the skeleton of what the robot does in autonomous and teleop.
- Other files in main/cpp control specific subsystems on the robot.
- The Include folder contains class headers laying out the functions of each subsystem.
- The file Include/Robot.h includes all of the library files used by the code base.
- The Include/Constants folder contains all of the useful numbers that must be changed to tune the robot.

## Standards in the code base
1. Robot-oriented coordinates are measured with positive X facing away, Y facing left, and Z facing upwards. The origin is centered on the chassis (x/y) and is on the ground (z). 
2. Robot-oriented coordinates are measured from the perspective of the blue driver station with positive X facing away, Y facing left, and Z facing upwards. The origin is in the close right corner of the field.
3. Positive rotation of the robot is (generally) counterclockwise positive with 0 degrees facing in the positive X direction.
