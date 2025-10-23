package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    private final DcMotorEx shootLeft;
    private final DcMotorEx shootRight;
    private final DcMotorEx intake;
    private final DcMotorEx conveyor;

    private final CRServo servo;

    public Shooter(HardwareMap hardwareMap) {
        shootLeft = hardwareMap.get(DcMotorEx.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotorEx.class, "shootRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");

        shootLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shootLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shootRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shootLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Tune velocity following
        shootLeft.setVelocityPIDFCoefficients(100,0.05,0,13);
        shootRight.setVelocityPIDFCoefficients(1,1,1,1);

        servo = hardwareMap.get(CRServo.class, "servo");

        servo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class SpinUpWheels implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //TODO: Add velocity following
            shootLeft.setVelocity(100);
            shootRight.setVelocity(100);

            return false;
        }
    }
    public Action spinUpWheels() {
        return new SpinUpWheels();
    }

    private class Shoot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPower(1);
            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return true;
            }
            servo.setPower(0);

            return false;
        }
    }

    public Action shoot() {
        return new Shoot();
    }

    public class Intake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            conveyor.setPower(1);
            intake.setPower(1);

            return false;
        }
    }
    public Action intake() {
        return new Intake();
    }
}