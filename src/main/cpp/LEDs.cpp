#include "LEDs.h"

/**
 * Instantiates a two motor elevator lift
 */
LEDLights::LEDLights(frc::PWMSparkMax *lights_)
{
  lights = lights_;
  SetLED();
}

void LEDLights::SetLED()
{
  SetLED(fire);
}

void LEDLights::SetLED(LEDLights::lightEffect color)
{
  lights->Set(lightEffectIDs[color]);
}
