#include "LEDs.h"

/**
 * Instantiates a two motor elevator lift
 */
LEDLights::LEDLights(frc::PWMSparkMax *lights_)
{
  lights = lights_;
  lightEffects["fire"] = -0.59;
  lightEffects["red"] = 0.61;
  lightEffects["orange"] = 0.65;
  lightEffects["yellow"] = 0.69;
  lightEffects["lime"] = 0.73;
  lightEffects["green"] = 0.77;
  lightEffects["blue"] = 0.81;
  lightEffects["purple"] = 0.91;
  SetLED("fire");
}

void LEDLights::SetLED()
{
  SetLED("fire");
}

void LEDLights::SetLED(std::string color)
{
  lights->Set(lightEffects[color]);
}
