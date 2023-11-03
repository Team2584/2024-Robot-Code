#include "PhotonTagSwerve.h"

PhotonTagSwerve::PhotonTagSwerve()
    : SwerveDrive(),
      camera{CAMERA_ONE_NAME}
{
}

void PhotonTagSwerve::ResetTagOdometry()
{
}