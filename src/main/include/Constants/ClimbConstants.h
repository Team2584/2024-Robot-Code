#ifndef CLIMB_CONSTANTS_H
#define CLIMB_CONSTANTS_H

namespace ClimbConstants {

inline const auto MaxHeight = 0.56_m;
inline const auto MinHeight = 0.03_m;

namespace Linear{
inline const auto m_KD  = 0;
inline const auto m_KP  = 30;
inline const auto m_KI  = 0;
inline const auto m_POS_ERROR  = 0.03_m;
inline const auto m_VELOCITY_ERROR = INFINITY;
inline units::meters_per_second_t kMaxVelocity = 0.1_mps;
inline units::meters_per_second_squared_t kMaxAcceleration = 0.75_mps_sq;
}

namespace Rotation{
inline const auto m_KD  = 0;
inline const auto m_KP  = 4;
inline const auto m_KI  = 0;
inline const auto m_ROTATION_TOLERACE  = 0.3_m; 
inline const auto m_VELOCITY_TOLERACE = INFINITY;
inline units::radians_per_second_t kMaxVelocity = units::radians_per_second_t{2};
inline units::radians_per_second_squared_t kMaxAcceleration = units::radians_per_second_squared_t{0.75};
}

inline const auto BasePctUp = 0.3;
inline const auto BasePctDown =  0.3;

inline constexpr auto gearRatioValue = 1.0/25.0;
inline constexpr auto diameterValue = 0.0508; //meters (GUESS)
inline constexpr auto CLIMB_CONVERSION_FACTOR = gearRatioValue * (3.14159365 * diameterValue);
}

#define CLIMB_MOTOR_L 23
#define CLIMB_MOTOR_R 24

#define CLIMB_LEFT_STOP_PORT 9
#define CLIMB_RIGHT_STOP_PORT 6

//Convention: Up = motor spin positive
//Right = positive

#endif