#ifndef FIELD_CONSTANTS_H // Ensures that this header file is only compiled once
#define FIELD_CONSTANTS_H

// List of april tag's IDs and their position on the field in comparison to origin
inline const std::vector<frc::AprilTag> FIELD_TAG_LAYOUT = {
    {0, Pose3d(1_m, 0_m, 1.15_m, Rotation3d(0_deg, 0_deg, 180_deg))},
    {1, Pose3d(5_m, 5_m, 5_m, Rotation3d())}};
inline const AprilTagFieldLayout APRIL_TAGS = AprilTagFieldLayout(FIELD_TAG_LAYOUT, 54_ft, 27_ft); // frc field is 54 ft by 27 ft

inline const Translation3d SPEAKER_POSITION = {0.5_m, 5.5_m, 2_m}; // TODO MAY BE INCORRECT, PLEASE CHECK SOMEONE

#endif // FIELD_CONSTANTS_H