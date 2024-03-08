//Candle Control 
#include <ctre/phoenix/led/CANdleLedStripType.h>
#include <ctre/phoenix/led/VBatOutputMode.h>
#include <ctre/phoenix/led/CANdleConfiguration.h>

//Animations
#include <ctre/phoenix/led/Animation.h>
#include <ctre/phoenix/led/ColorFlowAnimation.h>
#include <ctre/phoenix/led/LarsonAnimation.h>
#include <ctre/phoenix/led/RainbowAnimation.h>
#include <ctre/phoenix/led/SingleFadeAnimation.h>
#include <ctre/phoenix/led/StrobeAnimation.h>
#include "ctre/phoenix/led/BaseTwoSizeAnimation.h"
#include <ctre/phoenix/led/FireAnimation.h>
#include <ctre/phoenix/led/TwinkleAnimation.h>
#include <ctre/phoenix/led/TwinkleOffAnimation.h>
#include <ctre/phoenix/led/RgbFadeAnimation.h>
#include <ctre/phoenix/led/SingleFadeAnimation.h>


#include "Robot.h"
#include "Constants/LEDConstants.h"

#ifndef CANDLE_LED_H
#define CANDLE_LED_H

class Color {

    public:

         int red;
         int green;
         int blue;

         Color(int _red, int _green, int _blue) {
            red = _red;
            green = _green;
            blue = _blue;
        }

        /**
         * Highly imperfect way of dimming the LEDs. It does not maintain color or
         * accurately adjust perceived brightness.
         *
         * @param dimFactor
         * @return The dimmed color
         */
         Color dim(double dimFactor) {
            int newRed = (int) (std::clamp(red * dimFactor, 0.0, 200.0));
            int newGreen = (int) (std::clamp(green * dimFactor, 0.0, 200.0));
            int newBlue = (int) (std::clamp(blue * dimFactor, 0.0, 200.0));

            return Color(newRed, newGreen, newBlue);
    }
};

// Team colors
const Color orange = Color(255, 25, 0);
const Color black = Color(0, 0, 0);

// Game piece colors
const Color yellow = Color(242, 60, 0);
const Color purple = Color(184, 0, 185);

// Indicator colors
const Color white = Color(255, 230, 220);
const Color green = Color(56, 209, 0);
const Color blue = Color(8, 32, 255);
const Color red = Color(255, 0, 0);

using namespace ctre::phoenix::led;

class CandleController {

public:

    CANdle candle{LightsConstants::CANDLE_PORT};
    
    CandleController() {
        CANdleConfiguration candleConfiguration;
        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType::RGB;
        candleConfiguration.brightnessScalar = 1.0;
        candleConfiguration.vBatOutputMode = VBatOutputMode::Modulated;
        candle.ConfigAllSettings(candleConfiguration);
    }

    struct LEDSegment {

        CANdle* candle;

        int startIndex;
        int segmentSize;
        int animationSlot;

        LEDSegment(int startIndex, int segmentSize, int animationSlot, CANdle* _candle) : startIndex(startIndex), segmentSize(segmentSize), animationSlot(animationSlot), candle{_candle} {}

        void setColor(Color color) {
            clearAnimation();
            candle->SetLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
        }

        void setAnimation(BaseStandardAnimation animation) {
            candle->Animate(animation, animationSlot);
        }
        void setAnimation(BaseTwoSizeAnimation animation) {
            candle->Animate(animation, animationSlot);
        }

        void fullClear() {
            clearAnimation();
            disableLEDs();
        }

        void clearAnimation() {
            candle->ClearAnimation(animationSlot);
        }

        void disableLEDs() {
            setColor(black);
        }

        void setFlowAnimation(Color color, double speed) {
            setAnimation(ColorFlowAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, ColorFlowAnimation::Direction::Forward, startIndex));
        }

        void setFadeAnimation(Color color, double speed) {
            setAnimation(SingleFadeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        void setBandAnimation(Color color, double speed) {
            setAnimation(LarsonAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, LarsonAnimation::BounceMode::Front, 3, startIndex));
        }

        void setStrobeAnimation(Color color, double speed) {
            setAnimation(StrobeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        void setRainbowAnimation(double speed) {
            setAnimation(RainbowAnimation(1, speed, segmentSize, false, startIndex));
        }

        void setFireAnimation(double speed, double burnspeed, double coolspeed, bool isreversed){
            setAnimation(FireAnimation(1, speed, segmentSize, burnspeed, coolspeed, isreversed, startIndex));
        }

        void setTwinkleAnimation(Color color, double speed){
            setAnimation(TwinkleAnimation(color.red, color.green, color.blue,0,speed,segmentSize,ctre::phoenix::led::TwinkleAnimation::Percent100,startIndex));
        }

        void setRGBFadeAnimation(double speed){
            setAnimation(RgbFadeAnimation(1,speed,segmentSize,startIndex));
        }

        void setSingleFadeAnimation(Color color, double speed){
            setAnimation(SingleFadeAnimation(color.red, color.green, color.blue,0,speed,segmentSize,startIndex));
        }

    };

    LEDSegment BatteryIndicator{0,4,0, &candle};
    LEDSegment PhotonIndicator{4,2,-1, &candle};
    LEDSegment VisionIndicator{6,1,-1, &candle};
    LEDSegment DriverStationIndicator{7,1,-1, &candle};
    LEDSegment MainLEDStrip{8,LightsConstants::STRIP_LENGTH,1, &candle};

    void FullClear() {
        BatteryIndicator.fullClear();
        PhotonIndicator.fullClear();
        VisionIndicator.fullClear();
        DriverStationIndicator.fullClear();
        MainLEDStrip.setColor(red);
    }

    void ClearSegment(LEDSegment segment) {
        segment.clearAnimation();
        segment.disableLEDs();
    }

    void setBrightness(double percent) {
        candle.ConfigBrightnessScalar(percent, 100);
    }

};

class LightsSubsystem : public CandleController {

    public:

    frc::PowerDistribution *m_PDH;

    Timer notePickUpTimer;
    bool didStrobeGreen = false;

    LightsSubsystem(frc::PowerDistribution *_m_PDH)
    :   CandleController(),
        m_PDH{_m_PDH}
    {
        FullClear();
    }

    void UpdateSubsystemLEDS(){
       BatteryIndicator.setColor((m_PDH->GetVoltage() > 11.5 ? green : red));
       DriverStationIndicator.setColor((DriverStation::IsDSAttached() ? green : red));
    }

    void SetIdle(){
        MainLEDStrip.setFlowAnimation(red, 0.4);
    }

    void SetDriving(){
        MainLEDStrip.setFadeAnimation(red, 0.8);
    }

    void SetStrobeGreen(){
        MainLEDStrip.setStrobeAnimation(green, 0.1);
    }

    void SetStrobeBlue(){
        MainLEDStrip.setStrobeAnimation(blue, 0.02);
    }

    void SetStopped(){
        MainLEDStrip.setColor(red);
    }

    void SetEstopped(){
        MainLEDStrip.setSingleFadeAnimation(white, 0.2);
    }

    void SetHaveNote(){
        if(!didStrobeGreen){
            didStrobeGreen = true;
            notePickUpTimer.Restart();
        }
        if(notePickUpTimer.HasElapsed(LightsConstants::STROBE_TIME)){
            SetDriving();
        }
        else{
            SetStrobeGreen();
        }
    }



    void NoLongerHaveNote(){
        didStrobeGreen = false;
    }

    

};

#endif