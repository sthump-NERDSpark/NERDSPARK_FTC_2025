package org.firstinspires.ftc.teamcode.Util;

public class TrapezoidalProfile {
    private final double distance;
    private final double maxVel;
    private final double maxAccel;
    private final int sign;

    private final boolean triangular;
    private final double tAccel, tCruise, tTotal, dAccel, vPeak;

    public TrapezoidalProfile(double distance, double maxVel, double maxAccel) {
        this.sign = distance >= 0 ? 1 : -1;
        this.distance = Math.abs(distance);
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;

        // time to reach max velocity
        double t1 = maxVel / maxAccel;
        double d1 = 0.5 * maxAccel * t1 * t1;

        if (2 * d1 > this.distance) {
            // Triangular profile
            triangular = true;
            vPeak = Math.sqrt(maxAccel * this.distance);
            tAccel = vPeak / maxAccel;
            tCruise = 0.0;
            dAccel = 0.5 * maxAccel * tAccel * tAccel;
            tTotal = 2 * tAccel;
        } else {
            // Trapezoidal profile
            triangular = false;
            vPeak = maxVel;
            tAccel = t1;
            dAccel = d1;
            double dCruise = this.distance - 2 * dAccel;
            tCruise = dCruise / vPeak;
            tTotal = 2 * tAccel + tCruise;
        }
    }

    public double getTotalTime() {
        return tTotal;
    }

    /**
     * Compute desired position, velocity, acceleration at time t.
     */
    public State calculate(double t) {
        double pos, vel, acc;

        if (triangular) {
            if (t < tAccel) {
                acc = maxAccel;
                vel = acc * t;
                pos = 0.5 * acc * t * t;
            } else if (t < 2 * tAccel) {
                double tau = t - tAccel;
                acc = -maxAccel;
                vel = vPeak - maxAccel * tau;
                pos = dAccel + vPeak * tau - 0.5 * maxAccel * tau * tau;
            } else {
                acc = 0;
                vel = 0;
                pos = distance;
            }
        } else {
            if (t < tAccel) {
                acc = maxAccel;
                vel = acc * t;
                pos = 0.5 * acc * t * t;
            } else if (t < tAccel + tCruise) {
                acc = 0;
                vel = vPeak;
                pos = dAccel + vPeak * (t - tAccel);
            } else if (t < tTotal) {
                double tau = t - (tAccel + tCruise);
                acc = -maxAccel;
                vel = vPeak - maxAccel * tau;
                pos = dAccel + (distance - 2 * dAccel) + vPeak * tau - 0.5 * maxAccel * tau * tau;
            } else {
                acc = 0;
                vel = 0;
                pos = distance;
            }
        }

        return new State(sign * pos, sign * vel, sign * acc);
    }

    /** Container for setpoint data */
    public static class State {
        public final double position;
        public final double velocity;
        public final double acceleration;
        public State(double pos, double vel, double acc) {
            this.position = pos;
            this.velocity = vel;
            this.acceleration = acc;
        }
    }
}

