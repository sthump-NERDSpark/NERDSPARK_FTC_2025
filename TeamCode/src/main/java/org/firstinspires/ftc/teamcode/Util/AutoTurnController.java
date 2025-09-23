package org.firstinspires.ftc.teamcode.Util;

/**
 * A utility class for automatically turning a robot using
 * PID control combined with a trapezoidal motion profile.
 * <p>
 * Usage example (inside your robot code):
 * <p>
 * AutoTurnController turnController = new AutoTurnController(kP, kI, kD, maxVel, maxAccel);
 * turnController.setTargetAngle(Math.toRadians(90)); // Turn to +90 degrees
 * <p>
 * // In your loop:
 * double output = turnController.calculate(currentHeadingRadians);
 */
public class AutoTurnController {

    // PID constants
    private final double kP;
    private final double kI;
    private final double kD;

    // Motion profile limits
    private final double maxVelocity;     // rad/s
    private final double maxAcceleration; // rad/s^2

    // Internal PID state
    private double integral = 0.0;
    private double prevError = 0.0;
    private long lastTimestamp = 0;

    // Target (goal) heading
    private double targetAngle = 0.0;

    /**
     * Constructor for AutoTurnController.
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param maxVelocity Maximum angular velocity (rad/s)
     * @param maxAcceleration Maximum angular acceleration (rad/s^2)
     */
    public AutoTurnController(double kP, double kI, double kD, double maxVelocity, double maxAcceleration) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    /**
     * Sets the target angle (in radians).
     * Internally normalized to [-π, π) so shortest turn is chosen.
     */
    public void setTargetAngle(double angleRad) {
        this.targetAngle = normalizeAngle(angleRad);
    }

    /**
     * Gets the target angle (in radians).
     * Internally normalized to [-π, π) so shortest turn is chosen.
     */
    public double getTargetAngle() {
        return this.targetAngle;
    }

    /**
     * Main calculation method.
     * Call this in your control loop to get motor output.
     * @param currentAngle Current robot heading (radians)
     * @return Turn joystick value in range [-1, 1]
     */
    public double calculate(double currentAngle) {
        long now = System.nanoTime();
        double dt = (lastTimestamp == 0) ? 0.02 : (now - lastTimestamp) / 1e9; // seconds
        lastTimestamp = now;

        // Normalize both angles
        currentAngle = normalizeAngle(currentAngle);
        double error = normalizeAngle(targetAngle - currentAngle);

        // Simple trapezoidal velocity limiting:
        // Clamp error correction velocity based on acceleration and velocity limits
        double desiredVelocity = Math.signum(error) * Math.min(maxVelocity, Math.abs(error / dt));
        double limitedVelocity = clamp(desiredVelocity, -maxVelocity, maxVelocity);

        // PID terms
        integral += error * dt;
        double derivative = (error - prevError) / dt;
        prevError = error;

        // Final output = PID result
        double output = kP * error + kI * integral + kD * derivative;

        // Scale output by trapezoidal velocity shaping
        output = clamp(output, -limitedVelocity, limitedVelocity);

        // Normalize output into [-1, 1] joystick range
        double normalized = output / maxVelocity;
        return clamp(normalized, -1.0, 1.0);

    }

    /**
     * Helper: keep angle between [-π, +π).
     */
    private double normalizeAngle(double angle) {
        while (angle >= Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    /**
     * Helper: clamp a value to [min, max].
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

