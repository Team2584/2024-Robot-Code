#include "Robot.h"

#ifndef TOF_H_H
#define TOF_H_H

class DistanceOffsetTOF : private frc::TimeOfFlight{

    private:

        double toflastSpeed = 0;
        double toflastHeight = 0;
        double tofAllowedSigma = INFINITY;
        units::meter_t tofOffset = 0_m;

    public:

        DistanceOffsetTOF(int CANID, units::meter_t offset, double allowedSigma = INFINITY)
        : TimeOfFlight(CANID)
        {
            tofOffset = offset;
            tofAllowedSigma = allowedSigma;
            SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
            SetRangeOfInterest(4,4,12,12);
        }

        units::meter_t GetTof(){
            units::millimeter_t tofreading{GetRange()};
            double toferror = GetRangeSigma();

            if(toferror <= tofAllowedSigma)
                return tofreading - tofOffset;
            else {
                return 0_m;
            }
        }

};

#endif