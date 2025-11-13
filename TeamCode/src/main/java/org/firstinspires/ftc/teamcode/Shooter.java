package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter {
    private LimelightManager limelight;
    private boolean shootSeqActive = false;
    private final ElapsedTime shootTimer = new ElapsedTime();

    public enum ShooterActions {
        Intake,
        SpinUpWheels,
        Shoot,
        ShooterOFF,
        IntakeOFF,
        IntakeReverse, NoAction
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

        shootRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shootLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shootLeft.setVelocityPIDFCoefficients(100,0.05,0,13);
        shootRight.setVelocityPIDFCoefficients(100,0.05,0,13);

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
            case IntakeReverse: IntakeReverse();; break;
        }
   }

    private void SpinUpWheels() {


        Double Ty = limelight.getTy();

        double shooterVelocity;
        if (Ty == null)
        {
            shooterVelocity = 1000;
        } else if (Ty < - 10) {
            shooterVelocity = 1425;
        } else {
            shooterVelocity = 3.5579 * Ty * Ty - 17.578 * Ty + 709.34;
        }

   //     shootLeft.setVelocity(shooterVelocity);
   //     shootRight.setVelocity(shooterVelocity);
        shootLeft.setVelocity(1000);
        shootRight.setVelocity(1000);
    }

    private void Shoot() {
    //    intake.setPower(0);
    //    conveyor.setPower(1);
        double deltaTime = 500;
        double firstTime = 100;
        double secondTime = firstTime + deltaTime;
        double thirdTime = secondTime + deltaTime;
        double resetTime = thirdTime + deltaTime;

        if (!shootSeqActive) {
            shootSeqActive = true;
            shootTimer.reset();
        }
        Double Ty = limelight.getTy();

        double shooterVelocity;

        if (Ty == null)
        {
            shooterVelocity = 1000;
        } else if (Ty < - 10) {
            shooterVelocity = 1425;
        } else {
            shooterVelocity = 3.5579 * Ty * Ty - 17.578 * Ty + 709.34;
        }
        //   shootLeft.setVelocity(shooterVelocity);
        //   shootRight.setVelocity(shooterVelocity);
        shootLeft.setVelocity(1000);
        shootRight.setVelocity(1000);

        conveyor.setPower(1);

        double timer = shootTimer.milliseconds();
        if (timer >= firstTime && timer < secondTime) {
            servo.setPosition(0.128);
        }
        else if (timer >= secondTime && timer < thirdTime) {
            servo.setPosition(0.201);
        }
        else if (timer >= thirdTime && timer < resetTime) {
            servo.setPosition(0.24);
        }
        else if (timer >= resetTime) {
            servo.setPosition(0.060);
            currentAction = ShooterActions.ShooterOFF;
            shootSeqActive = false;
        }

    }
    private void Intake() {
        shootLeft.setVelocity(0);
        shootRight.setVelocity(0);
        servo.setPosition(0.060);

        conveyor.setPower(1);
        intake.setPower(1);
    }

    private void ShooterOff() {
        shootLeft.setVelocity(0);
        shootRight.setVelocity(0);
        conveyor.setPower(0);
    }

    private void IntakeOff() {
        intake.setPower(0);
        conveyor.setPower(0);
    }

    private void IntakeReverse() {
        intake.setPower(-1);
        conveyor.setPower(-1);
    }
}