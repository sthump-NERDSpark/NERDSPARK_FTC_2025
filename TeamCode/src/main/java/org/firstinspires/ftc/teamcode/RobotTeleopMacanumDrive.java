package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.PID;

@TeleOp(name = "Main Teleop for Mecanum Drive", group = "Robot")
public class RobotTeleopMacanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Change new Pose2d to match where you start out of auto
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        double commandDegrees = 0;
        PID turnController = new PID(0.022, 0, 0.0000005);
        Shooter shooter = new Shooter(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.right_stick_x;
            //double rx = gamepad1.left_stick_x;

            if (gamepad1.left_stick_x > 0.15) {
                commandDegrees -= 0.5;
            } else if (gamepad1.left_stick_x < -0.15) {
                commandDegrees += 0.5;
            }

            if(gamepad1.dpad_down) {
                if (Math.toDegrees(drive.localizer.getPose().heading.toDouble()) < 0) {
                    commandDegrees = -180;
                } else {
                    commandDegrees = 180;
                }
            } else if (gamepad1.dpad_up) {
                commandDegrees = 0;
            } else if (gamepad1.dpad_left) {
                commandDegrees = 90;
            } else if (gamepad1.dpad_right) {
                commandDegrees = -90;
            }

            telemetry.addData("Commanded Degrees: ", commandDegrees);
            telemetry.addData("Current Degrees: ", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

            double rx = turnController.calculate(Math.toDegrees(drive.localizer.getPose().heading.toDouble()), commandDegrees);
            telemetry.addData("Motor Command: ", rx);

            if (gamepad1.a) {
                shooter.setAction(Shooter.ShooterActions.SpinUpWheels);
            }
            if (gamepad1.x) {
                shooter.setAction(Shooter.ShooterActions.Shoot);
            }
            while (gamepad1.b) {
                shooter.setAction(Shooter.ShooterActions.Intake);
            }
            shooter.updateAction();

            telemetry.addData("Shooter left velo", shooter.shootLeft.getVelocity());
            telemetry.addData("Shooter right velo", shooter.shootRight.getVelocity());
            telemetry.addData("Lower",0);
            telemetry.addData("Upper", 1000);
            telemetry.update();

            drive.updatePoseEstimate();
            double botHeading = drive.localizer.getPose().heading.toDouble();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), -rx));
        }
    }
}