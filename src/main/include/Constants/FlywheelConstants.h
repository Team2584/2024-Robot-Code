#define FLYWHEEL_MOTOR_1 15
#define FLYWHEEL_MOTOR_2 16

//(these need to be set, IN RPS)
inline constexpr auto kShooterFreeRPS = 5300_tr / 1_s;
inline constexpr auto kShooterTargetRPS = 4000_tr / 1_s;
inline constexpr auto kShooterToleranceRPS = 50_tr / 1_s;

inline constexpr double kP = 1;
inline constexpr double kI = 0;
inline constexpr double kD = 0;

//THESE CONSTANTS ARE GUESSES (taken from WPILIB), must be determined empirically
//find using sysld: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html#introduction-to-system-identification
//or, rewrite all units and use https://www.reca.lc/flywheel based on flywheel diam/distance desired
inline constexpr auto kS = 0.05_V;
inline constexpr auto kV =  12.0_V / kShooterFreeRPS; 

inline constexpr double kEncoderDistancePerRotation = 0.5;

inline constexpr double kFeederSpeed = 50;

#define FLYWHEEL_BASE_PERCENT 60