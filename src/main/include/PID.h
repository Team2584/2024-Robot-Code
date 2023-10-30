#include <frc/controller/PIDController.h>
#include "Math.h"

/**
 * A PID loop allows motors to efficiently reach a certain point known as the setpoint.
 */
class PID
{
private:
    frc::PIDController pidController; /* the basic WPIlib PID Controller */
    double maxSpeed, minSpeed;        /* maximum and mininimum magnitude of values returned by the PID controller */

public:
    /**
     * Constructor for a PID Controller
     *
     * @param P The kP constant
     * @param I The kI constant
     * @param D The kD constant
     * @param maximumIntegral The maximum impact your I result can have on the system
     * @param minimumSpeed The minimum magnitude of the value returned
     * @param maximumSpeed The maximum magnitude of the value returned
     * @param positionTolerance The allowable error in the PID loop before finished
     * @param velocityTolerance The maxmimum velocity that can be experienced when PID loop is considered finished
     */
    PID(double P, double I, double D, double maximumIntegral, double minimumSpeed, double maximumSpeed, double positionTolerance, double velocityTolerance)
        : pidController{P, I, D}
    {
        pidController.SetIntegratorRange(-1 * maximumIntegral, maximumIntegral);
        pidController.SetTolerance(positionTolerance, velocityTolerance);
        maxSpeed = maximumSpeed;
        minSpeed = minimumSpeed;
    }

    /**
     * Constructor for a PID Controller
     *
     * @param P The kP constant
     * @param I The kI constant
     * @param D The kD constant
     */
    PID(double P, double I, double D)
        : pidController{P, I, D}
    {
        PID(P, I, D, 1, 0, 1, 0, 0);
    }

    void ResetPIDLoop()
    {
        pidController.Reset();
    }

    /* This function calculates the intended speed of a motor to reach a setpoint.
     * It is expected to be called once per 20ms loop.
     */
    double Calculate(double measurement, double setPoint)
    {
        // Calculate the PID loop and ensure it is within the maximum speed
        double intendedSpeed = std::clamp(pidController.Calculate(measurement, setPoint), -1 * maxSpeed, maxSpeed);

        // Ensure the intended speed is above the minimum speed
        if (fabs(intendedSpeed) < minSpeed)
            intendedSpeed = minSpeed * sgn(intendedSpeed);

        return intendedSpeed;
    }

    bool PIDFinished()
    {
        return pidController.AtSetpoint();
    }
};