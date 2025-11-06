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
        Shoot,
        ShooterOFF,
        IntakeOFF,
        NoAction
    }
    private ShooterActions currentAction = ShooterActions.NoAction;

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

        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.REVERSE);
    }

   public void setAction(ShooterActions action) {
        this.currentAction = action;
   }

   public void updateAction() {
        switch (currentAction) {
            case Shoot: Shoot(); break;
            case SpinUpWheels: SpinUpWheels(); break;
            case Intake: Intake(); break;
            case ShooterOFF: ShooterOff(); break;
            case IntakeOFF: IntakeOff(); break;
        }
   }

    private void SpinUpWheels() {
        shootLeft.setVelocityPIDFCoefficients(100,0.05,0,13);
        shootRight.setVelocityPIDFCoefficients(100,0.05,0,13);
        shootLeft.setVelocity(1000);
        shootRight.setVelocity(1000);
    }

    private void Shoot() {
        intake.setPower(0);
        conveyor.setPower(1);
        shootLeft.setVelocityPIDFCoefficients(100,0.05,0,13);
        shootRight.setVelocityPIDFCoefficients(100,0.05,0,13);
        shootLeft.setVelocity(1000);
        shootRight.setVelocity(1000);

        servo.setPosition(0.128);
        Wait(750);
        servo.setPosition(0.201);
        Wait(1000);
        servo.setPosition(0.24);
        Wait(750);
        servo.setPosition(0.058);
        conveyor.setPower(0);
        shootLeft.setVelocity(0);
        shootRight.setVelocity(0);
        currentAction = ShooterActions.Intake;
    }

    private void Intake() {
        shootLeft.setVelocity(0);
        shootRight.setVelocity(0);
        servo.setPosition(0.058);

        conveyor.setPower(1);
        intake.setPower(1);
    }

    private void ShooterOff() {
        shootLeft.setVelocity(0);
        shootRight.setVelocity(0);
    }

    private void IntakeOff() {
        intake.setPower(0);
        conveyor.setPower(0);
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