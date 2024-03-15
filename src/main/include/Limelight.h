#include "Robot.h"
#include "constants/LimelightConstants.h"

//Limelight API Docs: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api

#ifndef LIMELIGHT_H 
#define LIMELIGHT_H

class Limelight
{

public:

    enum LimelightPipeline{NOTES, APRILTAGS};
    std::shared_ptr<nt::NetworkTable> visionTable;
    nt::NetworkTableInstance networkTableInstance;
    nt::BooleanTopic noteInViewTopic; //Whether the limelight has any valid targets (0 or 1)
    nt::BooleanSubscriber noteInViewSubscriber;    
    nt::DoubleTopic notePosTopic_x; //LL2: -29.8 to 29.8 degrees)
    nt::DoubleSubscriber notePosSubscriber_x; 
    nt::DoubleTopic notePosTopic_y; //LL2: -24.85 to 24.85 degrees)
    nt::DoubleSubscriber notePosSubscriber_y;
    nt::DoubleArrayTopic tagPosTopic; // Position of robot based on april tags
    nt::DoubleArraySubscriber tagPosSubscriber;

private:

    Transform3d limelightPose; /* The Position and rotation of the camera on the robot */

public:

    Limelight(std::shared_ptr<nt::NetworkTable> limelightTable)
    :   visionTable{limelightTable},
        networkTableInstance{nt::NetworkTableInstance::GetDefault()},
        noteInViewTopic{visionTable->GetDoubleTopic("tv")},
        noteInViewSubscriber{noteInViewTopic.Subscribe({})},
        notePosTopic_x{visionTable->GetDoubleTopic("tx")},
        notePosSubscriber_x{notePosTopic_x.Subscribe({})},
        notePosTopic_y{visionTable->GetDoubleTopic("ty")},
        notePosSubscriber_y{notePosTopic_y.Subscribe({})},
        tagPosTopic{visionTable->GetDoubleArrayTopic("botpose_wpiblue")},
        tagPosSubscriber{tagPosTopic.Subscribe({})},
        limelightPose{frc::Translation3d{LimelightConstants::xPosOffset, LimelightConstants::yPosOffset, LimelightConstants::zPosOffset}, 
                      frc::Rotation3d{LimelightConstants::xRotOffset, LimelightConstants::yRotOffset, LimelightConstants::zRotOffset}}
    {
        // limelight table should be something like: nt::NetworkTableInstance::GetDefault().GetTable("limelight") - called in constructor
        // change the table name from "limelight" to the network table name your limelight is putting values in (default is "limelight")
        networkTableInstance.StartServer();
        visionTable->PutNumber("pipeline",0); //Set default vision pipeline
        visionTable->PutNumber("ledMode", 3); //limelight LEDs oFf (Blinding people isn't funny)
    }

    /**
     * @brief Set if the limelight should track notes or april tags (it can only do one at once)
    */
    void SetLimelightPipeline(LimelightPipeline limelightPipeline)
    {
        visionTable->PutNumber("pipeline", limelightPipeline);
    }

    LimelightPipeline GetLimelightPipeline()
    {
        return ((int) (visionTable->GetNumber("pipeline", 0)));
    }

    /**
     * @brief Get the pixel location of the center of the note in the left-right direction
    */
    double GetNoteTx(){ return notePosSubscriber_x.Get(); }
    
    /**
     * @brief Get the pixel location of the center of the note in the up-down direction
    */
    double GetNoteTy(){ notePosSubscriber_y.Get(); }

    /**
     * @brief Get the translation from the robot to the note
     * @return A translation representing the distance from the robot origin to the object
    */
    Translation2d GetNotePose(){

        units::degree_t xtargetdeg = units::degree_t{notePosSubscriber_x.Get()};
        units::degree_t ytargetdeg = units::degree_t{notePosSubscriber_y.Get()};

        units::radian_t xrotationrad = xtargetdeg - limelightPose.Rotation().X();
        units::radian_t yrotationrad = ytargetdeg - limelightPose.Rotation().Y();

        units::length::meter_t distanceFromLimelightToGoalX = (limelightPose.Translation().X())/units::math::tan(xrotationrad);
        units::length::meter_t distanceFromLimelightToGoalY = (limelightPose.Translation().Y())/units::math::tan(yrotationrad);

        return Translation2d(distanceFromLimelightToGoalX,distanceFromLimelightToGoalY);

    }

    /**
     * @brief Get the pose of the robot based on limelight's april tag detection
    */
    Pose3d GetTagBasedRobotPose()
    {   
        Translation3d position = Translation3d(units::meter_t{tagPosSubscriber.Get()[0]}, units::meter_t{tagPosSubscriber.Get()[1]}, units::meter_t{tagPosSubscriber.Get()[2]});
        Rotation3d rotation = Rotation3d(units::degree_t{tagPosSubscriber.Get()[3]}, units::degree_t{tagPosSubscriber.Get()[4]}, units::degree_t{tagPosSubscriber.Get()[5]});
        return Pose3d(position, rotation);
    }

};

#endif