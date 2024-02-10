#include "Robot.h"
#include "constants/LEDConstants.h"
#include <frc/motorcontrol/PWMSparkMax.h>
#include <iostream>

class LEDLights{
    
    /* !!!!!!!!!!
    To add more effects:
    1. Go to page 14-17 of https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf and find your effect
    2. Add an element to lightEffect under the name of the effect
    3. Add the roboRIO SPARK Value of the effect to the lightEffectIDs array
    !!!!!!!!!! */
    public:enum lightEffect{fire, red, orange, yellow, lime, green, blue, purple};
    private:float lightEffectIDs[100] = {-0.59, 0.61, 0.65, 0.69, 0.73, 0.77, 0.81, 0.91};






    private:
        int currentLightingEffect;
        frc::PWMSparkMax* lights;

    public:
        void SetLED(lightEffect color);
        void SetLED();

        float getCurrentLEDCode();
        int getCurrentLEDEffectID();

        PWMSparkMax* getLightController();

        LEDLights(int port);
};