package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Proportional Integral Derivative Controller w/ Low pass filter and anti-windup
 * This is specifically made for auto turning
 */
public class PID {
    private final double m_Kp;
    private final double m_Ki;
    private final double m_Kd;

    private double lastReference;
    private double previousFilterEstimate;
    private double integralSum = 0;
    private double lastError = 0;

    public PID(double Kp, double Ki, double Kd) {
        this.m_Kp = Kp;
        this.m_Ki = Ki;
        this.m_Kd = Kd;
    }

    public double calculate(double reference, double position) {
        // Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        boolean setPointIsNotReached = !(Math.abs(position - reference) < 0.5);

        if (setPointIsNotReached) {
            // calculate the error
            double error = reference - position;

            double errorChange = (error - lastError);

            // filter out high frequency noise to increase derivative performance
            double a = 0.8; // a can be anything from 0 < a < 1
            double currentFilterEstimate = (a * previousFilterEstimate) + (1- a) * errorChange;
            previousFilterEstimate = currentFilterEstimate;

            // rate of change of the error
            double derivative = currentFilterEstimate / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            // max out integral sum
            double maxIntegralSum = 100;
            if (integralSum > maxIntegralSum) {
                integralSum = maxIntegralSum;
            }

            if (integralSum < -maxIntegralSum) {
                integralSum = -maxIntegralSum;
            }

            // reset integral sum upon setpoint changes
            if (reference != lastReference) {
                integralSum = 0;
            }

            lastError = error;

            lastReference = reference;

            // reset the timer for next time
            timer.reset();

            return ((m_Kp * error) + (m_Ki * integralSum) + (m_Kd * derivative));
        }

        return 0;
    }

    //Return the current reference position based on the given motion profile times, maximum acceleration,
    // velocity, and current time
    public double motion_profile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {
        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            return distance;
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * Math.pow(elapsed_time, 2);
        } else if (elapsed_time < deceleration_time) { // if we're cruising
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        } else { // if we're decelerating
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * Math.pow(deceleration_time, 2);
        }
    }
}
