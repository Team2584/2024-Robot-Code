#include "Robot.h"
#include "constants/LEDConstants.h"
#include <frc/motorcontrol/PWMSparkMax.h>

class LEDLights{
    public:
        std::map<std::string, float> lightEffects;
        frc::PWMSparkMax* lights;

        float currentLightEffect = -0.59;

        void SetLED(std::string color);
        void SetLED();

        LEDLights(frc::PWMSparkMax *lights_);
};