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

    private final ElapsedTime timer;
    private double lastTime;

    public PID(double Kp, double Ki, double Kd) {
        this.m_Kp = Kp;
        this.m_Ki = Ki;
        this.m_Kd = Kd;
        timer = new ElapsedTime();
        lastTime = timer.seconds();
    }

    public double calculate(double reference, double position) {
        ElapsedTime timer = new ElapsedTime();

        boolean setPointIsNotReached = !(Math.abs(position - reference) < 1);

        if (setPointIsNotReached) {
            double error;
            if (Math.abs(position - reference) < 180) {
                error = position - reference;
            } else {
                if (position - reference < 0) {
                    error = position - (reference - 360);
                } else {
                    error = position - (reference + 360);
                }
            }

            // calculate the error
            //double error = reference - position;

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

            double power = (m_Kp * error) + (m_Ki * integralSum) + (m_Kd * derivative);
            double maxVal = Math.max(Math.abs(power), 1.0);

            return power / maxVal;
        }

        return 0;
    }

    public double calculatePosition(double reference, double position) {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        boolean setPointIsNotReached = !(Math.abs(position - reference) < 0.5);

        if (setPointIsNotReached) {
            // calculate the error
            double error = reference - position;

            double errorChange = (error - lastError);

            // filter out high frequency noise to increase derivative performance
            double a = 0.6; // a can be anything from 0 < a < 1
            double currentFilterEstimate = (a * previousFilterEstimate) + (1- a) * errorChange;
            previousFilterEstimate = currentFilterEstimate;

            // rate of change of the error
            double derivative = currentFilterEstimate / dt;

            // sum of all error over time
            integralSum = integralSum + (error * dt);

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

            double power = (m_Kp * error) + (m_Ki * integralSum) + (m_Kd * derivative);
            double maxVal = Math.max(Math.abs(power), 1.0);

            return power / maxVal;
        }

        return 0;
    }
}
