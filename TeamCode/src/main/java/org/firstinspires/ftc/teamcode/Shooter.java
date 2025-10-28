package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    public enum ShooterActions {
        Intake,
        SpinUpWheels,
        Shoot
    }
    private ShooterActions currentAction;

    public final DcMotorEx shootLeft;
    public final DcMotorEx shootRight;
    private final DcMotorEx intake;
    private final DcMotorEx conveyor;

    private final Servo servo;

    public Shooter(HardwareMap hardwareMap) {
        shootLeft = hardwareMap.get(DcMotorEx.class, "shootLeft");
        shootRight = hardwareMap.get(DcMotorEx.class, "shootRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        conveyor = hardwareMap.get(DcMotorEx.class, "conveyor");

        shootLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shootRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shootLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shootLeft.setVelocityPIDFCoefficients(100,0.05,0,13);
        shootRight.setVelocityPIDFCoefficients(100,0.05,0,13);

        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.REVERSE);
    }

   public void setAction(ShooterActions action) {
        this.currentAction = action;
   }

   public void updateAction() {
        switch (currentAction) {
            case Shoot: Shoot();
            case SpinUpWheels: SpinUpWheels();
            case Intake: Intake();
            default: Zero();
        }
   }

    private void SpinUpWheels() {
        shootLeft.setVelocity(900);
        shootRight.setVelocity(900);
    }

    private void Shoot() {
        intake.setPower(0);
        conveyor.setPower(0.25);

        servo.setPosition(0.116);
        Wait(500);
        servo.setPosition(0.187);
        Wait(500);
        servo.setPosition(0.24);
        Wait(100);
        servo.setPosition(0.045);
    }

    private void Intake() {
        shootLeft.setVelocity(0);
        shootRight.setVelocity(0);
        servo.setPosition(0);

        conveyor.setPower(1);
        intake.setPower(1);
    }

    private void Zero() {
        shootLeft.setVelocity(0);
        shootRight.setVelocity(0);
        servo.setPosition(0);
        conveyor.setPower(0);
        intake.setPower(0);
    }

    /**
     * Time should be in milliseconds
     */
    private void Wait(double time) {
        ElapsedTime timer = new ElapsedTime();

        while (true) {
            if (timer.milliseconds() >= time) {
                break;
            }
        }
    }
}