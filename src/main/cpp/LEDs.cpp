#include "LEDs.h"

/* !!!!!!!!!!
In Robot.cpp:
  #include "LEDs.h"
  
In order to use SetLED:
  In Robot.cpp:
    Ex: LEDLights *lightStrip{0};
    Do: LEDLights *NAME{INT PORT};
  
  To call:
    Ex: lightStrip->SetLED(LEDLights::fire);
    Do: NAME->SetLED(LEDLights::EFFECT NAME)
!!!!!!!!!! */
void LEDLights::SetLED(int color)
{
  lights->Set(lightEffectIDs[color]);
  currentLightingEffect = color;
}

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