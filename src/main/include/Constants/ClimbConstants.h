namespace ClimbConstants {

inline const double MaxHeight = 100;
inline const double MinHeight = 0;

inline const double m_linear_KD  = 0;
inline const double m_linear_KP  = 4;
inline const double m_linear_KI  = 0;
inline const double m_linear_KIMAX  = 0;
inline const double m_linear_POS_ERROR  = 0.1; //~3deg
inline const double m_linear_VELOCITY_ERROR = 1;
inline const double m_linear_MAX_SPEED  = 0.3;
inline const double m_linear_MIN_SPEED  = 0.1;

inline const double m_rotation_KD  = 0;
inline const double m_rotation_KP  = 4;
inline const double m_rotation_KI  = 0;
inline const double m_rotation_KIMAX  = 0;
inline const double m_rotation_ROT_ERROR  = 0.1; //~3deg
inline const double m_rotation_VELOCITY_ERROR = 1;
inline const double m_rotation_MAX_SPEED  = 0.3;
inline const double m_rotation_MIN_SPEED  = 0.1;

inline const double BasePctUp = 0.05;
inline const double BasePctDown =  0.05;

}

#define CLIMB_MOTOR_L 23
#define CLIMB_MOTOR_R 24

//Convention: Up = motor spin positive
//Right = positive