#include "Robot.h"
#include "frc/XboxController.h"
#include <thread>
#include <chrono>

using namespace frc;

//Wrapper class for WPILIB's XboxController class to handle driver feedback. 
class RumbleXboxController : public XboxController {

    private:

        bool rumbling = false;

    public:

        RumbleXboxController(int port) 
        : XboxController(port)
        { 
        }

        void HaveNoteRumble(){
            rumble(2,200,100);
        }
        void ShotNoteRumble(){
            rumble(2,200,100);
        }
        void ReadyActionRumble(){
            rumble(2,200,100);
        }

    private: 
    
        /**
         * @brief Rumbles the controller for a set time
         * This function makes use of C++ threads. Threads used improperly WILL crash the robot's code due to spawning infinitly. Be careful. 
         * A check is used in this case to make sure a thread is not currently running before starting a new thread. 
        */
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