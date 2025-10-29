package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ServoPosition extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo left = hardwareMap.get(Servo.class, "leftKick");
        Servo center = hardwareMap.get(Servo.class, "centerKick");
        Servo right = hardwareMap.get(Servo.class, "rightKick");

        left.setDirection(Servo.Direction.FORWARD);
        center.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            left.setPosition(0.85);
            center.setPosition(0.85);
            right.setPosition(0.85);

            ElapsedTime timer = new ElapsedTime();

            while (true) {
                if (timer.milliseconds() >= 1000) {
                    break;
                }
            }

            left.setPosition(0.65);
            center.setPosition(0.65);
            right.setPosition(0.65);

            timer.reset();

            while (true) {
                if (timer.milliseconds() >= 1000) {
                    break;
                }
            }
        }
    }
}
