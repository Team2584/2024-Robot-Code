#ifndef FLYWHEEL_CONSTANTS_H // Ensures that this header file is only compiled once
#define FLYWHEEL_CONSTANTS_H

#define FLYWHEEL_MOTOR_1 15
#define FLYWHEEL_MOTOR_2 16
#define FLYWHEEL_ANGLING_MOTOR 30
#define FLYWHEEL_MAG_ENCODER_PORT 8
#define FLYWHEEL_MAG_ENCODER_OFFSET -0.1525;

namespace FlywheelConstants {

//(these need to be set, IN RPS)
inline constexpr auto kShooterFreeRPS = 113_tr / 1_s; //~neo vortex rpm
inline constexpr auto kShooterTargetRPS = 70_tr / 1_s;               
inline constexpr auto kShooterToleranceRPS = 15_tr / 1_s;
inline constexpr auto kShooterTargetRPS_S = 15_tr / 1_s / 1_s;

inline double KP = 0.000000001; //make this ~zero if flywheel rampsup too hard
inline double KI = 0;
inline double KD = 0; 

//This (guess) kV = 0.10619, actual vortex kV = 0.10432
//kS (min voltage to max motor spin) will need to be found using sysid https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
inline constexpr auto KS = 0.05_V;
inline constexpr auto KV =  12.0_V / kShooterFreeRPS; 

inline constexpr double kFeederSpeed = 50;

inline constexpr auto FLYWHEEL_BASE_PERCENT = 0.8;

namespace Angler {
// Flywheel Angler PID Values
inline constexpr auto KS = 0_V;
inline constexpr auto KG = 0.1625_V;
inline constexpr auto KV = 0.4_V * 1_s / 1_rad; //0.88
inline constexpr auto KP = 15;
inline constexpr auto KI = 0;
inline constexpr auto KI_MAX = 0; 
inline constexpr auto KD = 0;
inline constexpr auto POS_TOLERANCE = 0.03;
inline constexpr auto VELOCITY_TOLERANCE = 5; 
inline constexpr auto MIN_SPEED = 0;
inline constexpr auto MAX_SPEED = 10; 
}

/* Autonomous Constants */
inline constexpr auto shotTime = 0.75_s;
}



#endif //FLYWHEEL_CONSTANTS_H