package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.PID;

@TeleOp(name = "Main Teleop for Mecanum Drive", group = "Robot")
public class RobotTeleopMacanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Change new Pose2d to match where you start out of auto
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        double commandDegrees = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        PID turnController = new PID(0, 0, 0);
        double rx;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.right_stick_x;
            //double rx = gamepad1.left_stick_x;

            // First attempt at incorporating manual turning with auto turn
            if (gamepad1.left_stick_x > 0.05) {
                commandDegrees += 0.5;
            }

            // First tested attempt at auto turn. Runs all the time and constantly corrects gyro position
            if(gamepad1.dpad_down) {
                commandDegrees = 180;
            } else if (gamepad1.dpad_up) {
                commandDegrees = 0;
            } else if (gamepad1.dpad_left) {
                commandDegrees = 90;
            } else if (gamepad1.dpad_right) {
                commandDegrees = -90;
            }

            telemetry.addData("Commanded Degrees: ", commandDegrees);
            telemetry.addData("Current Degrees: ", drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            rx = turnController.calculate(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), commandDegrees);
            telemetry.update();

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
        }
    }
}