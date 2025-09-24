package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.AutoTurnController;

@TeleOp(name = "Main Teleop for Mecanum Drive", group = "Robot")
public class RobotTeleopMacanumDrive extends LinearOpMode {
    public static MecanumDrive.Params PARAMS = new MecanumDrive.Params();

    @Override
    public void runOpMode() {
        // Change new Pose2d to match where you start out of auto


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
       //        double degree = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        AutoTurnController turnController = new AutoTurnController(0.06, 0, 0,
                drive.defaultTurnConstraints.maxAngVel, drive.defaultTurnConstraints.maxAngAccel);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.right_stick_x;
            //double botHeading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            drive.updatePoseEstimate();
            double botHeading = drive.localizer.getPose().heading.toDouble();
            //double rx = gamepad1.left_stick_x;

            // First attempt at incorporating manual turning with auto turn
            if (gamepad1.left_stick_x > 0.05) {
                turnController.setTargetAngle(turnController.getTargetAngle() + Math.toRadians(5));
            }

            // First tested attempt at auto turn. Runs all the time and constantly corrects gyro position
//            if(gamepad1.dpad_down) {
//                degrees = 180;
//            } else if (gamepad1.dpad_up) {
//                degrees = 0;
//            } else if (gamepad1.dpad_left) {
//                degrees = 90;
//            } else if (gamepad1.dpad_right) {
//                degrees = -90;
//            }
//
//            telemetry.addData("Commanded Degrees: ", degrees);
//            telemetry.addData("Current Degrees: ", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//
//            rx = drive.AutoTurn(degrees, telemetry);
//            telemetry.update();

            // Second attempt at auto turn. Runs all the time and constantly corrects gyro position
            if(gamepad1.dpad_down && botHeading > 0) {
                turnController.setTargetAngle(Math.toRadians(180));
            } else if (gamepad1.dpad_down && botHeading <= 0) {
                turnController.setTargetAngle(Math.toRadians(-180));
            } else if (gamepad1.dpad_up) {
                turnController.setTargetAngle(Math.toRadians(0));
            } else if (gamepad1.dpad_left) {
                turnController.setTargetAngle(Math.toRadians(90));
            } else if (gamepad1.dpad_right) {
                turnController.setTargetAngle(Math.toRadians(-90));
            }

            double rx = turnController.calculate(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            telemetry.addData("Commanded Degrees: ", Math.toDegrees(turnController.getTargetAngle()));
            telemetry.addData("Current Degrees: ", Math.toDegrees(botHeading));
            telemetry.update();

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                drive.imu.resetYaw();
            }

            //double botHeading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), rx));
        }
    }
}