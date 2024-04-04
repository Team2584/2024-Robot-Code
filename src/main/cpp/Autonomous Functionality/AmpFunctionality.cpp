#include "Autonomous Functionality/AmpFunctionality.h"

AutonomousAmpingController::AutonomousAmpingController(SwerveDriveAutonomousController *swerveDrive_, NoteController *noteController_) 
    : ampTimer{}
{
    swerveDrive = swerveDrive_;
    noteController = noteController_;
}

/**
 * @brief Reset Values/Prepare for the Drive to Amp Sequence
*/
void AutonomousAmpingController::BeginDriveToAmp(AllianceColor allianceColor){
    centeredOnAmp = false;
    drivingIntoAmp = false;
}

/**
 * @brief Drive to the Pre-Defined Amp Pose based off Tag Odometry for a specific alliance color
 * @param AllianceColor The Correct AllianceColor Enumerator for the amp you need to drive to
 * @note Call the BeginDriveToAmp Function each start of this loop
*/
bool AutonomousAmpingController::DriveToAmp(AllianceColor allianceColor){

    if (!centeredOnAmp) 
    {
        if (allianceColor == AllianceColor::BLUE)
            centeredOnAmp = swerveDrive->DriveToPose(ElevatorConstants::BLUE_AMP_SCORING_POSITION, PoseEstimationType::TagBased);
        else 
            centeredOnAmp = swerveDrive->DriveToPose(ElevatorConstants::RED_AMP_SCORING_POSITION, PoseEstimationType::TagBased);
        return false;
    }

    if (!drivingIntoAmp)
    {
        drivingIntoAmp = true;
        ampTimer.Restart();
    }

    if (ampTimer.Get() < ElevatorConstants::ampDriveTime)
    {
        swerveDrive->swerveDrive->DriveSwervePercentTagOriented(0, 0.3, 0);
        return false;
    }
    
    swerveDrive->swerveDrive->DriveSwervePercent(0,0,0);
    return true;
}
