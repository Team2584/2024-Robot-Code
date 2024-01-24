#include "Math.h"
#include <frc/controller/PIDController.h>

/**
 * A PID loop allows motors to efficiently reach a certain point known as the setpoint.
 * This class mainly functions as a wrapper for the wpi pid controller adding minimal functionality.
 * Hopefully, however, it will make it easier to maintain the same code base if WPIlib changes in the future.
 */
class PID
{
protected:
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

    /**
     * Reinstantiate PID Controller
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
    void ChangeConstants(double P, double I, double D, double maximumIntegral, double minimumSpeed, double maximumSpeed, double positionTolerance, double velocityTolerance)
    {
        pidController.SetPID(P, I, D);
        pidController.SetIntegratorRange(-1 * maximumIntegral, maximumIntegral);
        pidController.SetTolerance(positionTolerance, velocityTolerance);
        maxSpeed = maximumSpeed;
        minSpeed = minimumSpeed;
        
        ResetPIDLoop();
    }

    /**
     * Enables continuous input. Rather then using the max and min input range as constraints, 
     * it considers them to be the same point and automatically calculates the shortest route to the setpoint.
     * 
     * @param minimumInput the minimum value the PID loop can reach
     * @param maximumInput the maximum value the PID loop can reach
     */
    void EnableContinuousInput(double minimumInput, double maximumInput)
    {
        pidController.EnableContinuousInput(minimumInput, maximumInput);
    }

    /* 
     * If you do not call the Calculate function every loop, this function can be called to restart the PID Controller.
     */
    void ResetPIDLoop()
    {
        pidController.Reset();
    }

    /* 
     * This function calculates the intended speed of a motor to reach a setpoint.
     * It is expected to be called once per 20ms loop.
     */
    double Calculate(double measurement, double setPoint)
    {
        // Calculate the PID loop and ensure it is within the maximum speed
        double intendedSpeed = std::clamp(pidController.Calculate(measurement, setPoint), -1 * maxSpeed, maxSpeed);

        // Ensure the intended speed is above the minimum speed
        if (fabs(intendedSpeed) < minSpeed)
            intendedSpeed = minSpeed * sgn(intendedSpeed);

        // If done, stop driving the motor
        if (PIDFinished())
            intendedSpeed = 0;

        return intendedSpeed;
    }

    /* 
     * This function returns true when the error of the pid loop is below the tolerance, i.e. it has reached the goal.
     */
    bool PIDFinished()
    {
        return pidController.AtSetpoint();
    }

    double GetPIDSetpoint(){
        return pidController.GetSetpoint();
    }

    double GetPIDAllowedError(){
        return pidController.GetPositionTolerance();
    }

};