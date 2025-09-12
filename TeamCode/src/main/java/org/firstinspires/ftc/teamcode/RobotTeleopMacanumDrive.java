package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class RobotTeleopMacanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FLMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BLMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FRMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BRMotor");

        // TODO: Change motor directions based on your drive train
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // TODO: Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT)
        );
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.right_stick_x;
            double rx = gamepad1.left_stick_x;

            if(gamepad1.dpad_down) {
                Actions.runBlocking(drive.new TurnAction(new TimeTurn(
                        drive.localizer.getPose(),
                        Util.toRadians(180),
                        drive.defaultTurnConstraints)
                ));
            } else if (gamepad1.dpad_up) {
                Actions.runBlocking(drive.new TurnAction(new TimeTurn(
                        drive.localizer.getPose(),
                        Util.toRadians(0),
                        drive.defaultTurnConstraints)
                ));
            } else if (gamepad1.dpad_left) {
                Actions.runBlocking(drive.new TurnAction(new TimeTurn(
                        drive.localizer.getPose(),
                        Util.toRadians(90),
                        drive.defaultTurnConstraints)
                ));
            } else if (gamepad1.dpad_right) {
                Actions.runBlocking(drive.new TurnAction(new TimeTurn(
                        drive.localizer.getPose(),
                        Util.toRadians(270),
                        drive.defaultTurnConstraints)
                ));
            }

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}