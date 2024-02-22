#include "Robot.h"
#include "constants/LimelightConstants.h"

//Limelight API Docs: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api

class Limelight
{

public:

    nt::NetworkTableInstance networkTableInstance;
    std::shared_ptr<nt::NetworkTable> visionTable;
    nt::BooleanTopic noteInViewTopic;
    nt::BooleanSubscriber noteInViewSubscriber;    
    nt::DoubleTopic notePosTopic_x;
    nt::DoubleSubscriber notePoseSubscriber_x;
    nt::DoubleTopic notePosTopic_y;
    nt::DoubleSubscriber notePoseSubscriber_y;

private:

    Transform3d limelightPose; /* The Position and rotation of the camera on the robot */

public:

    Limelight(std::shared_ptr<nt::NetworkTable> limelightTable)
    :   visionTable{limelightTable},
        networkTableInstance{nt::NetworkTableInstance::GetDefault()},
        noteInViewTopic{visionTable->GetDoubleTopic("tv")},
        noteInViewSubscriber{noteInViewTopic.Subscribe({})},
        notePosTopic_x{visionTable->GetDoubleTopic("tx")},
        notePoseSubscriber_x{notePosTopic_x.Subscribe({})},
        notePosTopic_y{visionTable->GetDoubleTopic("ty")},
        notePoseSubscriber_y{notePosTopic_y.Subscribe({})},
        limelightPose{frc::Translation3d{LimelightConstants::xPosOffset, LimelightConstants::yPosOffset, LimelightConstants::zPosOffset}, 
                      frc::Rotation3d{LimelightConstants::xRotOffset, LimelightConstants::yRotOffset, LimelightConstants::zRotOffset}}
    {
        // limelight table should be something like: nt::NetworkTableInstance::GetDefault().GetTable("limelight") - called in constructor
        // change the table name from "limelight" to the network table name your limelight is putting values in (default is "limelight")
        visionTable = limelightTable;
        networkTableInstance.StartServer();
        visionTable->PutNumber("pipeline",0); //Set default vision pipeline
        visionTable->PutNumber("ledMode", 3); //limelight LEDs of (Blinding people isn't funny)
    }
    
    Translation2d GetNotePose(){
        units::degree_t xtargetdeg = units::degree_t{notePoseSubscriber_x.Get()};
        units::degree_t ytargetdeg = units::degree_t{notePoseSubscriber_y.Get()};

        units::radian_t xrotationrad = xtargetdeg - limelightPose.Rotation().X();
        units::radian_t yrotationrad = ytargetdeg - limelightPose.Rotation().Y();

        units::length::meter_t distanceFromLimelightToGoalX = (limelightPose.Translation().X())/units::math::tan(xrotationrad);
        units::length::meter_t distanceFromLimelightToGoalY = (limelightPose.Translation().Y())/units::math::tan(yrotationrad);

        return Translation2d(distanceFromLimelightToGoalX,distanceFromLimelightToGoalY);

    }

};