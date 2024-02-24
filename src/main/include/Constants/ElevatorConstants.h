#include "Robot.h"

#ifndef ELEVATOR_CONSTANTS_H
#define ELEVATOR_CONSTANTS_H

#define ELEVATOR_MOTOR_PORT 25
#define AMP_MECH_PORT 26

namespace ElevatorConstants {
inline units::meters_per_second_t kMaxVelocity = 2_mps;
inline units::meters_per_second_squared_t kMaxAcceleration = 0.75_mps_sq;
inline double m_kP = 225;
inline double m_kI = 4;
inline double m_kD = 0.0;
inline units::volt_t m_kS = 1.1_V;
inline units::volt_t m_kG = 0_V;
inline auto m_kV = 1.3_V / 1_mps;
inline auto ALLOWABLE_ERROR_POS = 0.04_m;

inline auto ELEV_AMP = 0.53;
inline auto ELEV_LOW = 0;
inline auto ELEV_OUTTAKE = 0.1;
inline auto ELEV_TRAP = 0.65;

inline constexpr double gearRatioValue1 = 1.0/25.0;
inline constexpr double diameterValue1 = 0.0762; //meters
inline constexpr double ELEV_CONVERSION_FACTOR = gearRatioValue1 * (3.14159365 * diameterValue1);

namespace AmpMech{
    inline constexpr double AMP_SPEED_FROM_SELECTOR = -0.5;
    inline constexpr double AMP_SPEED_TO_SELECTOR = 0.5;
    inline constexpr double AMP_SPEED_DEPOSIT = -0.75;
}

// Autonomously Score Amp Constants
inline const Pose2d BLUE_AMP_SCORING_POSITION = {1.8415_m, 7.5216_m, Rotation2d(90_deg)};
inline const Pose2d RED_AMP_SCORING_POSITION = {14.7_m, 7.5216_m, Rotation2d(90_deg)};
inline constexpr auto ampDriveTime = 0.15_s;
}





#endif