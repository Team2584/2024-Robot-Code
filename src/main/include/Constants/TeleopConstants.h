//Customization Variabes (all in percent so the driver's weak brain don't get confused)
#define CONTROLLER_DEADBAND 0.125
#define TRIGGER_ACTIVATION_POINT 0.25
#define TRIGGER_DEACTIVATION_POINT 0.1

#define MAX_DRIVE_SPEED 0.6
#define MAX_SPIN_SPEED 0.6
#define SPEED_BOOST_DRIVE 0.9
#define SPEED_BOOST_SPIN 0.9
#define MAX_DRIVE_SPEED_SHOOT_ON_THE_MOVE 0.25 //was .1, .25
#define MAX_DRIVE_SPEED_CLIMB 0.3
#define MAX_SPIN_SPEED_CLIMB 0.3
inline constexpr auto DRIVE_SLEW_RATE  = 2.0; // Maximum change in drive percentage per second (i.e '2' means the drive will go from fully forward to fully reverse in 1 second)
inline constexpr auto SPIN_SLEW_RATE  = 2.0;

const inline auto SHOT_TIME = 1.25_s;
#define FLYWHEEL_IDLE_RPM 1500.0;

inline const double WRIST_LOW_INTAKE_CUTOFF = 0.07; // we only intake notes when the wrist is below this value

#define STARTING_DRIVE_HEADING  0.0