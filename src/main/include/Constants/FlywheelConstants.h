#define FLYWHEEL_MOTOR_1 15
#define FLYWHEEL_MOTOR_2 16
#define FLYWHEEL_ANGLING_MOTOR 17

//(these need to be set, IN RPS)
inline constexpr auto kShooterFreeRPS = 113_tr / 1_s; //neo vortex
inline constexpr auto kShooterTargetRPS = 70_tr / 1_s;
inline constexpr auto kShooterToleranceRPS = 5_tr / 1_s;

inline constexpr double kP = 0.01;
inline constexpr double kI = 0;
inline constexpr double kD = 0;

//This (guess) kV = 0.10619, actual vortex kV = 0.10432
//kS will need to be found using sysid https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
inline constexpr auto kS = 0.05_V;
inline constexpr auto kV =  12.0_V / kShooterFreeRPS; 

inline constexpr double kFeederSpeed = 50;

#define FLYWHEEL_BASE_PERCENT 60

// Flywheel Angler PID Values
#define ANGLER_KP 0.007
#define ANGLER_KI 0
#define ANGLER_KI_MAX 0 // In percent power
#define ANGLER_KD 0
#define ANGLER_TOLERANCE 3 // In Degrees
#define ANGLER_VELOCITY_TOLERANCE 0.5 // In percent power
#define ANGLER_MIN_SPEED 0 // In percent power
#define ANGLER_MAX_SPEED 1.0 // In percent power