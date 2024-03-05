#include "Robot.h"
#include "frc/XboxController.h"
#include <thread>
#include <chrono>

using namespace frc;

class RumbleXboxController : public XboxController {

public:

    bool rumbling = false;

    RumbleXboxController(int port) 
    : XboxController(port)
    { 

    }

    void rumble(int times, int mscdelay, int strengthpct) {
        if(!rumbling){
        // Start a new thread for rumbling
        double pct = strengthpct/100.0;
        std::thread rumbleThread([this, times, mscdelay, pct] {
            rumbling = true;
            for (int i = 0; i < times; i++) {

                // Rumble on
                SetRumble(frc::GenericHID::RumbleType::kLeftRumble, pct);
                SetRumble(frc::GenericHID::RumbleType::kRightRumble, pct);
                
                // Wait for a short duration
                std::this_thread::sleep_for(std::chrono::milliseconds(mscdelay)); 
                
                // Rumble off
                SetRumble(frc::GenericHID::RumbleType::kLeftRumble, 0);
                SetRumble(frc::GenericHID::RumbleType::kRightRumble, 0);

                // Wait for a short duration before next rumble
                std::this_thread::sleep_for(std::chrono::milliseconds(mscdelay)); 
            }
            rumbling = false;
        });

        // Detach the thread to allow it to run independently
        rumbleThread.detach();
        }
    }
    
};