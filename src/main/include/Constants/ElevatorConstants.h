#include "Robot.h"

#ifndef ELEVATOR_CONSTANTS_H
#define ELEVATOR_CONSTANTS_H

#define ELEVATOR_MOTOR_PORT 25
#define AMP_MECH_PORT 26

namespace ElevatorConstants {
inline units::meters_per_second_t kMaxVelocity = 1.05_mps;
inline units::meters_per_second_squared_t kMaxAcceleration = 0.75_mps_sq;
inline units::volt_t maxVoltsDown = 2_V;
inline double m_kP = 50.0; //36
inline double m_kI = 0.05;
inline double m_kD = 0.0;
inline units::volt_t m_kS = 0_V;
inline units::volt_t m_kG = 0.3_V; //BEFORE PULLEY CHANGE
inline auto m_kV = 1.3_V / 1_mps;
inline auto ALLOWABLE_ERROR_POS = 0.04_m;

inline auto ELEV_AMP = 0.53;
inline auto ELEV_LOW = 0;
inline auto ELEV_INTAKE = 0;
inline auto ELEV_OUTTAKE = 0.15;
inline auto ELEV_TRAP = 0.70;

inline auto ELEV_SOURCE = 0.60; //fake value


inline constexpr double gearRatioValue1 = 1.0/4.0;
inline constexpr double diameterValue1 = 0.0762 / 2.0; //meters
inline constexpr double ELEV_CONVERSION_FACTOR = gearRatioValue1 * (3.14159365 * diameterValue1);

namespace AmpMech{
    inline constexpr double AMP_SPEED_FROM_SELECTOR = -0.5;
    inline constexpr double AMP_SPEED_TO_SELECTOR = 0.5;
    inline constexpr double AMP_SPEED_DEPOSIT = -0.75;
}

namespace TimeOfFlight{
    inline constexpr double tofAllowedSigma = 1.75;
    inline constexpr auto tofOffset = 0.05_m;
    inline constexpr int tofCANID = 27;
}

// Autonomously Score Amp Constants
inline const Pose2d BLUE_AMP_SCORING_POSITION = {1.8415_m, 7.92_m, Rotation2d(90_deg)};
inline const Pose2d RED_AMP_SCORING_POSITION = {14.7_m, 7.92_m, Rotation2d(90_deg)};
inline constexpr auto ampDriveTime = 0.1_s;
}





#endif