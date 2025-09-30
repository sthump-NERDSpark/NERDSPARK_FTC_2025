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
        ElapsedTime timer = new ElapsedTime();

        boolean setPointIsNotReached = !(Math.abs(position - reference) < 1);

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
}
