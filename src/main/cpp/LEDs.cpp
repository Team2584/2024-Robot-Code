#include "LEDs.h"

LEDLights::LEDLights(int port)
{
  frc::PWMSparkMax lightController{port};
  lights = &lightController;
  SetLED();
}

void LEDLights::SetLED()
{
  SetLED(fire);
}

/* !!!!!!!!!!
In order to use SetLED:
#include "LEDs.h"
LEDLights::SetLED(LEDLights::lightEffect::fire);
!!!!!!!!!! */
void LEDLights::SetLED(LEDLights::lightEffect color)
{
  lights->Set(lightEffectIDs[color]);
}
