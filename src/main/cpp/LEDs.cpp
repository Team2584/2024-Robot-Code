
/* !!!!!!!!!!
In order to use SetLED:
LEDLights::SetLED(LEDLights::lightEffect::fire);
!!!!!!!!!! */
void LEDLights::SetLED(LEDLights::lightEffect color)
{
  lights->Set(lightEffectIDs[color]);
  currentLightingEffect = color;
}






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

float LEDLights::getCurrentLEDCode(){
  return lightEffectIDs[currentLightingEffect];
}

int LEDLights::getCurrentLEDEffectID(){
  return currentLightingEffect;
}

PWMSparkMax* LEDLights::getLightController(){
  return lights;
}