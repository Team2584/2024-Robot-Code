#ifndef ELEVATOR_CONSTANTS_H
#define ELEVATOR_CONSTANTS_H

#define ELEVATOR_MOTOR_PORT 25
#define AMP_MECH_PORT 26

namespace ElevatorConstants {
inline units::meters_per_second_t kMaxVelocity = 1.75_mps;
inline units::meters_per_second_squared_t kMaxAcceleration = 0.75_mps_sq;
inline double m_kP = 60;
inline double m_kI = 4;
inline double m_kD = 0.0;
inline units::volt_t m_kS = 1.1_V;
inline units::volt_t m_kG = 0_V;
inline auto m_kV = 1.3_V / 1_mps;
inline auto ALLOWABLE_ERROR_POS = 0.04_m;

inline auto ELEV_AMP = 0.57;
inline auto ELEV_LOW = 0;
inline auto ELEV_TRAP = 0.3;

inline constexpr double gearRatioValue1 = 1.0/25.0;
inline constexpr double diameterValue1 = 0.0762; //meters
inline constexpr double ELEV_CONVERSION_FACTOR = gearRatioValue1 * (3.14159365 * diameterValue1);
}





#endif