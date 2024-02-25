#ifndef CLIMB_CONSTANTS_H
#define CLIMB_CONSTANTS_H

namespace ClimbConstants {

inline const double MaxHeight = -0.56;
inline const double MinHeight = 0.03;

inline const double m_linear_KD  = 0;
inline const double m_linear_KP  = 100;
inline const double m_linear_KI  = 0;
inline const double m_linear_KIMAX  = 0.4;
inline const double m_linear_POS_ERROR  = 0.03;
inline const double m_linear_VELOCITY_ERROR = 10000;
inline const double m_linear_MAX_SPEED  = 0.5;
inline const double m_linear_MIN_SPEED  = 0.05;

inline const double m_rotation_KD  = 0;
inline const double m_rotation_KP  = 4;
inline const double m_rotation_KI  = 0;
inline const double m_rotation_KIMAX  = 0;
inline const double m_rotation_ROT_ERROR  = 0.3; 
inline const double m_rotation_VELOCITY_ERROR = 10000;
inline const double m_rotation_MAX_SPEED  = 0.3;
inline const double m_rotation_MIN_SPEED  = 0.05;

inline const double BasePctUp = 0.3;
inline const double BasePctDown =  0.3;

inline constexpr double gearRatioValue = 1.0/16.0;
inline constexpr double diameterValue = 0.0508; //meters (GUESS)
inline constexpr double CLIMB_CONVERSION_FACTOR = gearRatioValue * (3.14159365 * diameterValue);
}

#define CLIMB_MOTOR_L 23
#define CLIMB_MOTOR_R 24

#define CLIMB_LEFT_STOP_PORT 9
#define CLIMB_RIGHT_STOP_PORT 6

//Convention: Up = motor spin positive
//Right = positive

#endif