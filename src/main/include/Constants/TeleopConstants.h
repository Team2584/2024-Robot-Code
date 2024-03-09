//Customization Variabes (all in percent so the driver's weak brain don't get confused)
#define CONTROLLER_DEADBAND 0.125
#define TRIGGER_ACTIVATION_POINT 0.3
#define TRIGGER_DEACTIVATION_POINT 0.1

#define MAX_DRIVE_SPEED 0.6
#define MAX_SPIN_SPEED 0.6
#define SPEED_BOOST_DRIVE 0.9
#define SPEED_BOOST_SPIN 0.9
#define MAX_DRIVE_SPEED_SHOOT_ON_THE_MOVE 0.1
#define MAX_DRIVE_SPEED_CLIMB 0.3
#define MAX_SPIN_SPEED_CLIMB 0.3
#define DRIVE_SLEW_RATE 4_mps // Maximum change in drive percentage per second (i.e '2' means the drive will go from fully forward to fully reverse in 1 second)
#define SPIN_SLEW_RATE 4_mps // Not actually meters per second

const inline auto SHOT_TIME = 1_s;
#define FLYWHEEL_IDLE_RPM 1500.0;

#define STARTING_DRIVE_HEADING  0.0