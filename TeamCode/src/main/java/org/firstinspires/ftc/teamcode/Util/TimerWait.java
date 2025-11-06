package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TimerWait {
    private final ElapsedTime timer = new ElapsedTime();
    private double delayMilliseconds = 0;

    // Start a new wait (non-blocking)
    public void startWait(double milliseconds) {
        delayMilliseconds = milliseconds;
        timer.reset();
    }

    // Returns true once the delay has passed
    public boolean isDone() {
        return timer.milliseconds() >= delayMilliseconds;
    }

    // Optional: check progress (0.0 to 1.0)
    public double getProgress() {
        return Math.min(timer.seconds() / delayMilliseconds, 1.0);
    }

    // Optional: check remaining time
    public double getRemainingTime() {
        return Math.max(delayMilliseconds - timer.seconds(), 0);
    }
}