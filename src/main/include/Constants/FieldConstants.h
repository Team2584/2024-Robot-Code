#include "Robot.h"

#ifndef FIELD_CONSTANTS_H // Ensures that this header file is only compiled once
#define FIELD_CONSTANTS_H

// List of april tag's IDs and their position on the field in comparison to origin
inline const std::vector<frc::AprilTag> FIELD_TAG_LAYOUT = {
    {0, Pose3d(0.5_m, 5.5_m, 1.15_m, Rotation3d(0_deg, 0_deg, 180_deg))}, // For Testing, Delete Later
    {1, Pose3d(15.079471999999997_m, 0.24587199999999998_m, 1.355852_m, Rotation3d(0_deg, 0_deg, 120_deg))},
    {2, Pose3d(16.185134_m, 0.883666_m, 1.355852_m, Rotation3d(0_deg, 0_deg, 120_deg))},
    {3, Pose3d(16.579342_m, 4.982717999999999_m, 1.4511020000000001_m, Rotation3d(0_deg, 0_deg, 180_deg))},
    {4, Pose3d(16.579342_m, 5.547867999999999_m, 1.4511020000000001_m, Rotation3d(0_deg, 0_deg, 180_deg))},
    {5, Pose3d(14.700757999999999_m, 8.2042_m, 1.355852_m, Rotation3d(0_deg, 0_deg, 270_deg))},
    {6, Pose3d(1.8415_m, 8.2042_m, 1.355852_m, Rotation3d(0_deg, 0_deg, 270_deg))},
    {7, Pose3d(-0.038099999999999995_m, 5.547867999999999_m, 1.4511020000000001_m, Rotation3d(0_deg, 0_deg, 0_deg))},
    {8, Pose3d(-0.038099999999999995_m, 4.982717999999999_m, 1.4511020000000001_m, Rotation3d(0_deg, 0_deg, 0_deg))},
    {9, Pose3d(0.356108_m, 0.883666_m, 1.355852_m, Rotation3d(0_deg, 0_deg, 6_deg))},
    {10, Pose3d(1.4615159999999998_m, 0.24587199999999998_m, 1.355852_m, Rotation3d(0_deg, 0_deg, 60_deg))},
    {11, Pose3d(11.904726_m, 3.7132259999999997_m, 1.3208_m, Rotation3d(0_deg, 0_deg, 300_deg))},
    {12, Pose3d(11.904726_m, 4.49834_m, 1.3208_m, Rotation3d(0_deg, 0_deg, 60_deg))},
    {13, Pose3d(11.220196_m, 4.105148_m, 1.3208_m, Rotation3d(0_deg, 0_deg, 180_deg))},
    {14, Pose3d(5.320792_m, 4.105148_m, 1.3208_m, Rotation3d(0_deg, 0_deg, 0_deg))},
    {15, Pose3d(4.641342_m, 4.49834_m, 1.3208_m, Rotation3d(0_deg, 0_deg, 120_deg))},
    {16, Pose3d( 4.641342_m, 3.7132259999999997_m, 1.3208_m, Rotation3d(0_deg, 0_deg, 240_deg))},
    };
inline const AprilTagFieldLayout APRIL_TAGS = AprilTagFieldLayout(FIELD_TAG_LAYOUT, 54_ft, 27_ft); // frc field is 54 ft by 27 ft

inline const Translation3d SPEAKER_POSITION = {0.3_m, 5.5_m, 2.75_m}; // TODO MAY BE INCORRECT, PLEASE CHECK SOMEONE

#endif // FIELD_CONSTANTS_H