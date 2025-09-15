package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Robot: Main Teleop for Mecanum Drive", group = "Robot")
public class RobotTeleopMacanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Change new Pose2d to match where you start out of auto
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        double degrees = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double rx;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.right_stick_x;
            //double rx = gamepad1.left_stick_x;

            // First attempt at incorporating manual turning with auto turn
            if (gamepad1.left_stick_x > 0.05) {
                degrees = degrees + gamepad1.left_stick_x;
            }

            // First attempt at auto turn, never tested but may not be able to drive robot when auto turning
            // due to the function taking over direct motor power control
//            if(gamepad1.dpad_down) {
//                Actions.runBlocking(drive.new TurnAction(new TimeTurn(
//                        drive.localizer.getPose(),
//                        Math.toRadians(180),
//                        drive.defaultTurnConstraints)
//                ));
//            } else if (gamepad1.dpad_up) {
//                Actions.runBlocking(drive.new TurnAction(new TimeTurn(
//                        drive.localizer.getPose(),
//                        Math.toRadians(0),
//                        drive.defaultTurnConstraints)
//                ));
//            } else if (gamepad1.dpad_left) {
//                Actions.runBlocking(drive.new TurnAction(new TimeTurn(
//                        drive.localizer.getPose(),
//                        Math.toRadians(90),
//                        drive.defaultTurnConstraints)
//                ));
//            } else if (gamepad1.dpad_right) {
//                Actions.runBlocking(drive.new TurnAction(new TimeTurn(
//                        drive.localizer.getPose(),
//                        Math.toRadians(270),
//                        drive.defaultTurnConstraints)
//                ));
//            }

            // Second attempt at auto turn. Runs all the time and constantly corrects gyro position
            if(gamepad1.dpad_down) {
                degrees = 180;
            } else if (gamepad1.dpad_up) {
                degrees = 0;
            } else if (gamepad1.dpad_left) {
                degrees = 90;
            } else if (gamepad1.dpad_right) {
                degrees = -90;
            }

            rx = drive.AutoTurn(degrees);

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                drive.imu.resetYaw();
            }

            double botHeading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), rx));

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
//
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);
        }
    }
}