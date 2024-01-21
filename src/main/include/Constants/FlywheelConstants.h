#define FLYWHEEL_MOTOR_1 15
#define FLYWHEEL_MOTOR_2 16

inline constexpr auto kShooterFreeRPS = 5300_tr / 1_s;
inline constexpr auto kShooterTargetRPS = 4000_tr / 1_s;
inline constexpr auto kShooterToleranceRPS = 50_tr / 1_s;

inline constexpr double kP = 1;
inline constexpr double kI = 0;
inline constexpr double kD = 0;

// On a real robot the feedforward constants should be empirically determined;
// these are reasonable guesses.
inline constexpr auto kS = 0.05_V;
inline constexpr auto kV =  12.0_V / kShooterFreeRPS; 

inline constexpr double kFeederSpeed = 50;

#define FLYWHEEL_BASE_PERCENT 60