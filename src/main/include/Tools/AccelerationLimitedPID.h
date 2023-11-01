#include "PID.h"
#include <frc/filter/SlewRateLimiter.h>

class AccelerationLimitedPID : public PID {
private:
    frc::SlewRateLimiter slewRateLimiter; /* the basic WPIlib Slew Rate Limiter (i.e. acceleration controller) */
    double lastSpeed; /* the speed returned in the most recent Calculate call */

public:
    /**
     * Constructor for an acceleration limited PID Controller
     *
     * @param maximumNegativeAcceleration the maximum acceleration the PID loop can output in the negative direction (MUST BE A NEGATIVE NUMBER)
     * @param maximumPositiveAcceleration the maximum acceleration the PID loop can output in the positive direction (MUST BE A POSITIVE NUMBER)
     */
    AccelerationLimitedPID(double maximumNegativeAcceleration, double maximumPositiveAcceleration)
    : slewRateLimiter{maximumPositiveAcceleration, maximumNegativeAcceleration}
    {
        lastSpeed = 0;
    }

    /* 
     * This function calculates the intended speed of a motor to reach a setpoint.
     * It is expected to be called once per 20ms loop.
     */
    double Calculate(double measurement, double setPoint)
    {
        // Calculates the speed using a basic PID calculator
        double intendedSpeed = PID::Calculate(measurement, setPoint);

        // Limits the acceleration of the basic PID calculator
        lastSpeed = slewRateLimiter.Calculate(intendedSpeed);
        return lastSpeed;
    }

    bool PIDFinished()
    {
        // Checks if the basic PID loop is finished, i.e. the controller is within position tolerance
        bool basePIDFinished = PID::PIDFinished();

        // Checks if the acceleration PID loop is finished, i.e. the controller is within velocity tolerance
        bool accelerationLimitedPIDFinished = fabs(lastSpeed) < PID::velTolerance;

        // Returns true if both controllers are finished
        return basePIDFinished && accelerationLimitedPIDFinished;
    }
}