#include "Autonomous Functionality/AmpFunctionality.h"

AutonomousAmpingController::AutonomousAmpingController(SwerveDriveAutonomousController *swerveDrive_, NoteController *noteController_) 
    : ampTimer{}
{
    swerveDrive = swerveDrive_;
    noteController = noteController_;
}

void AutonomousAmpingController::BeginDriveToAmp(){
    centeredOnAmp = false;
    drivingIntoAmp = false;
}

bool AutonomousAmpingController::DriveToAmp(){
    if (!centeredOnAmp) 
    {
        centeredOnAmp = swerveDrive->DriveToPose(ElevatorConstants::AMP_SCORING_POSITION, PoseEstimationType::TagBased);
        return false;
    }

    if (!drivingIntoAmp)
    {
        drivingIntoAmp = true;
        ampTimer.Restart();
    }

    if (ampTimer.Get() < ElevatorConstants::ampDriveTime)
    {
        swerveDrive->swerveDrive->DriveSwervePercentTagOriented(0, -0.3, 0);
        return false;
    }
    
    swerveDrive->swerveDrive->DriveSwervePercent(0,0,0);
    return true;
}
