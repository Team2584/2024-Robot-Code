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
     * @param P The kP constant
     * @param I The kI constant
     * @param D The kD constant
     * @param maximumIntegral The maximum impact your I result can have on the system
     * @param minimumSpeed The minimum magnitude of the value returned
     * @param maximumSpeed The maximum magnitude of the value returned
     * @param positionTolerance The allowable error in the PID loop before finished
     * @param velocityTolerance The maxmimum velocity that can be experienced when PID loop is considered finished
     * @param maximumNegativeAcceleration the maximum acceleration the PID loop can output in the negative direction (MUST BE A NEGATIVE NUMBER)
     * @param maximumPositiveAcceleration the maximum acceleration the PID loop can output in the positive direction (MUST BE A POSITIVE NUMBER)
     */
    AccelerationLimitedPID(double P, double I, double D, double maximumIntegral, double minimumSpeed, double maximumSpeed, double positionTolerance, double velocityTolerance, double maximumNegativeAcceleration, double maximumPositiveAcceleration)
    :         PID(P, I, D, maximumIntegral, minimumSpeed, maximumSpeed, positionTolerance, velocityTolerance),
                slewRateLimiter{maximumPositiveAcceleration, maximumNegativeAcceleration}
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

    /* 
     * This function returns true when the error of the pid loop is below the tolerance, i.e. it has reached the goal.
     */
    bool PIDFinished()
    {
        // Checks if the basic PID loop is finished, i.e. the controller is within position tolerance
        bool basePIDFinished = PID::PIDFinished();

        // Checks if the acceleration PID loop is finished, i.e. the controller is within velocity tolerance
        bool accelerationLimitedPIDFinished = fabs(lastSpeed) < PID::velTolerance;

        // Returns true if both controllers are finished
        return basePIDFinished && accelerationLimitedPIDFinished;
    }
};