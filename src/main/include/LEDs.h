#include "Robot.h"
#include "constants/LEDConstants.h"
#include <frc/motorcontrol/PWMSparkMax.h>
#include <iostream>

class LEDLights{
    public:
        /* !!!!!!!!!!
        To add more effects:
        1. Go to page 14-17 of https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf and find your effect
        2. Add an element to lightEffect under the name of the effect
        3. Add the roboRIO SPARK Value of the effect to the lightEffectIDs array
        !!!!!!!!!! */
        enum lightEffect{fire, red, orange, yellow, lime, green, blue, purple};
        float lightEffectIDs[100] = {-0.59, 0.61, 0.65, 0.69, 0.73, 0.77, 0.81, 0.91};


        frc::PWMSparkMax* lights;
            
        float currentLightEffect = lightEffectIDs[lightEffect::fire];

        void SetLED(lightEffect color);
        void SetLED();

        LEDLights(frc::PWMSparkMax *lights_);
};