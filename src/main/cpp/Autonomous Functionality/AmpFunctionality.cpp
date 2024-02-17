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
        centeredOnAmp = swerveDrive->DriveToPose(AMP_SCORING_POSITION);
        return false;
    }

    if (!drivingIntoAmp)
    {
        drivingIntoAmp = true;
        ampTimer.Restart();
    }

    swerveDrive->DriveSwervePercent(0, 0.3, 0);
    
}
