package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Remember to STOP_AND_RESET the pivot encoders in auto init but not in teleop due to the possibility
 * that the starting position may not be exact
 */
public class Shooter {
    private DcMotorEx shootTop;
    private DcMotorEx shootBottom;
    public DcMotorEx pivotLeft;
    public DcMotorEx pivotRight;

    private Servo kickLeft;
    private Servo kickCenter;
    private Servo kickRight;

    private ColorSensor sensorLeft;
    private ColorSensor sensorCenter;
    private ColorSensor sensorRight;

    public Shooter(HardwareMap hardwareMap) {
        shootTop = hardwareMap.get(DcMotorEx.class, "shootTop");
        shootBottom = hardwareMap.get(DcMotorEx.class, "shootTop");
        pivotLeft = hardwareMap.get(DcMotorEx.class, "leftPivot");
        pivotRight = hardwareMap.get(DcMotorEx.class, "rightPivot");

        shootTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        shootTop.setDirection(DcMotorSimple.Direction.FORWARD);
        shootBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        pivotRight.setDirection(DcMotorSimple.Direction.REVERSE);

        shootTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        kickLeft = hardwareMap.get(Servo.class, "leftKick");
        kickCenter = hardwareMap.get(Servo.class, "centerKick");
        kickRight = hardwareMap.get(Servo.class, "rightKick");

        sensorLeft = hardwareMap.get(ColorSensor.class, "leftColor");
        sensorCenter = hardwareMap.get(ColorSensor.class, "centerColor");
        sensorRight = hardwareMap.get(ColorSensor.class, "rightColor");
    }

    public class AimAndSpinUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            return false;
        }
    }

    /**
     *This action aims the shooter and spins up the motors
     */
    public Action aimAndSpinUp() {
        return new AimAndSpinUp();
    }

    public class Shoot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            return false;
        }
    }

    /**
     *This action shoots the balls in a specified order
     */
    public Action shoot() {
        return new Shoot();
    }

    /**
     *This action aligns the robot to the goal, aims the shooter, and spins up the motors
     */
    public Action alignAndAim(Pose2d currPose) {
        double heading = Math.atan2(0,0) * (180/Math.PI);
        return new SequentialAction();
    }
}