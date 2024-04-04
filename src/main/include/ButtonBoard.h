#include "Robot.h"

class ButtonBoard : private GenericHID {

    public:

        ButtonBoard(int port)
        : GenericHID(port)
        {
        }

        bool GetAnglerTrimDec(){
            return GetRawButtonPressed(0);
        }

        bool GetAnglerTrimInc(){
            return GetRawButtonPressed(1);
        }

        bool GetFlywheelStopped(){
            return false;
        }

        bool GetAltIntake(){
            return false;
        }

        bool GetAltClimb(){
            return false;
        }

};