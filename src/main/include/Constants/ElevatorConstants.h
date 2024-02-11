#ifndef ELEVATOR_CONSTANTS_H
#define ELEVATOR_CONSTANTS_H

#define ELEVATOR_MOTOR_PORT 25
#define AMP_MECH_PORT 26

/*
//Holding Elevator at Position PID (copied from 2023 besides the //guess)
#define ELEVHOLDFF 0
#define ELEVKP 0.08
#define ELEVKI 0
#define ELEVKD 0
#define ELEVKIMAX 0.1
#define ALLOWABLE_ERROR_ELEV_POS 0.6
#define ALLOWABLE_ERROR_ELEV_VELOCITY 10 //guess
#define ELEVMAX_SPEED 0.9
#define ELEVMIN_SPEED 0 //guess
*/
inline units::meters_per_second_t e_kMaxVelocity = 1.75_mps;
inline units::meters_per_second_squared_t e_kMaxAcceleration = 0.75_mps_sq;
inline double e_kP = 60;
inline double e_kI = 4;
inline double e_kD = 0.0;
inline units::volt_t e_kS = 1.1_V;
inline units::volt_t e_kG = 0_V;
inline auto e_kV = 1.3_V / 1_mps;
inline auto ALLOWABLE_ERROR_ELEV_POS = 0.04_m;

inline auto ELEV_AMP = 0.57;
inline auto ELEV_LOW = 0;
inline auto ELEV_TRAP = 0.3;

inline constexpr double gearRatioValue1 = 1.0/25.0;
inline constexpr double diameterValue1 = 0.0762; //meters
inline constexpr double ELEV_CONVERSION_FACTOR = gearRatioValue1 * (3.14159365 * diameterValue1);

#endif