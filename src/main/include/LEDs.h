#include "Robot.h"
#include "constants/LEDConstants.h"
#include <frc/motorcontrol/PWMSparkMax.h>
#include <iostream>

class LEDLights{
    public:
        enum lightEffect{fire, red, orange, yellow, lime, green, blue, purple};
        float lightEffectIDs[8] = {-0.59, 0.61, 0.65, 0.69, 0.73, 0.77, 0.81, 0.91};
        frc::PWMSparkMax* lights;

        float currentLightEffect = lightEffectIDs[lightEffect::fire];

        void SetLED(lightEffect color);
        void SetLED();

        LEDLights(frc::PWMSparkMax *lights_);
};