#include "Robot.h"
#include "constants/LEDConstants.h"
#include <frc/motorcontrol/PWMSparkMax.h>
#include <iostream>

class LEDLights{

    /* !!!!!!!!!!
    To add more effects:
    1. Go to page 14-17 of https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf and find the effect you would like to add
    2. Add an element to the lightEffect enum (line 14) below with the name of the effect from the pdf (Can be simplifed, Ex: "Fire, Medium" -> "fire")
    3. Get the "roboRIO SPARK Value" from the pdf and add it to the lightEffectIDs float array line (line 15) below
    (Optional: format the enum and array so the floats and names first characters line up for better readablity)
    !!!!!!!!!! */
    public:enum lightEffect{       fire,  red,  orange, yellow, lime, green, blue, purple};
    private:float lightEffectIDs[100] = {-0.59, 0.61, 0.65,   0.69,   0.73, 0.77,  0.81, 0.91};






    private:
        int currentLightingEffect;
        frc::PWMSparkMax* lights;

    public:
        void SetLED(int color);
        void SetLED();

        float getCurrentLEDCode();
        int getCurrentLEDEffectID();

        PWMSparkMax* getLightController();

        LEDLights(int port);
};