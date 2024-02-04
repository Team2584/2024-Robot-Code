#define C_MAX_H 100
#define C_MIN_H 0

inline double c_KD  = 0;
inline double c_KP  = 4;
inline double c_KI  = 0;
inline double c_KIMAX  = 0;
inline double c_POS_ERROR  = 0.1; //~3deg
inline double c_VELOCITY_ERROR = 1;
inline double c_MAX_SPEED  = 0.3;
inline double c_MIN_SPEED  = 0.1;

inline double c_t_KD  = 0;
inline double c_t_KP  = 4;
inline double c_t_KI  = 0;
inline double c_t_KIMAX  = 0;
inline double c_t_ROT_ERROR  = 0.1; //~3deg
inline double c_t_VELOCITY_ERROR = 1;
inline double c_t_MAX_SPEED  = 0.3;
inline double c_t_MIN_SPEED  = 0.1;

#define CLIMB_MOTOR_L 23
#define CLIMB_MOTOR_R 24

#define CLIMB_PCT_UP 0.05
#define CLIMB_PCT_DOWN 0.05

#define CLIMB_LEFT_STOP_PORT 2
#define CLIMB_RIGHT_STOP_PORT 3

//Convention: Up = motor spin positive
//Right = positive