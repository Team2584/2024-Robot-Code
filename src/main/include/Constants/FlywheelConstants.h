#define FLYWHEEL_MOTOR_1 15
#define FLYWHEEL_MOTOR_2 16

//(these need to be set, IN RPS)
inline constexpr auto kShooterFreeRPS = 113_tr / 1_s; //~neo vortex rpm
inline constexpr auto kShooterTargetRPS = 70_tr / 1_s;                 
inline constexpr auto kShooterToleranceRPS = 15_tr / 1_s;
inline constexpr auto kShooterTargetRPS_S = 15_tr / 1_s / 1_s;

inline double f_kP = 0.000000001; //make this ~zero if flywheel rampsup too hard
inline constexpr double f_kI = 0;
inline constexpr double f_kD = 0; 

//This (guess) kV = 0.10619, actual vortex kV = 0.10432
//kS (min voltage to max motor spin) will need to be found using sysid https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
inline constexpr auto kS = 0.05_V;
inline constexpr auto kV =  12.0_V / kShooterFreeRPS; 

inline constexpr double kFeederSpeed = 50;

#define FLYWHEEL_BASE_PERCENT 60