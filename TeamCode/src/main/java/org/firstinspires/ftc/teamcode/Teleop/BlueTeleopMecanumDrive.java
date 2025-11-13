package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.LimelightManager;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Util.PID;

@TeleOp(name = "Blue Teleop for Mecanum Drive", group = "Robot")
public class BlueTeleopMecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Change new Pose2d to match where you start out of auto
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        double commandDegrees = 0;
        PID turnController = new PID(0.022, 0, 0.0000005);
        Shooter shooter = new Shooter(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");
        dashboard.startCameraStream(ll, 0);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        LimelightManager limelight = new LimelightManager(hardwareMap, telemetry, true);

        limelight.setPipeline(0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -(gamepad1.right_stick_y * 0.8); // Remember, Y stick value is reversed
            double x = -(gamepad1.right_stick_x * 0.8);
            //double rx = gamepad1.left_stick_x;

            if (gamepad1.left_stick_x > 0.15) {
                if (gamepad1.left_bumper) {
                    commandDegrees -= 5;
                } else {
                    commandDegrees -= 0.5;
                }
            } else if (gamepad1.left_stick_x < -0.15) {
                if (gamepad1.left_bumper) {
                    commandDegrees += 5;
                } else {
                    commandDegrees += 0.5;
                }
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
            if (gamepad1.y) {
                commandDegrees = -45;
            }

            telemetry.addData("Commanded Degrees: ", commandDegrees);
            telemetry.addData("Current Degrees: ", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

            double rx = turnController.calculate(Math.toDegrees(drive.localizer.getPose().heading.toDouble()), commandDegrees);
            telemetry.addData("Motor Command: ", rx);

            if (gamepad2.a) {
                shooter.setAction(Shooter.ShooterActions.SpinUpWheels);
            }
            if (gamepad2.x) {
                shooter.setAction(Shooter.ShooterActions.Shoot);
            }
            if (gamepad2.b) {
                shooter.setAction(Shooter.ShooterActions.Intake);
            }
            if (gamepad2.y) {
                shooter.setAction(Shooter.ShooterActions.IntakeReverse);
            }
            if (gamepad2.right_bumper) {
                shooter.setAction(Shooter.ShooterActions.ShooterOFF);
            }
            if (gamepad2.left_bumper) {
                shooter.setAction(Shooter.ShooterActions.IntakeOFF);
            }
            shooter.updateAction();

            drive.localizer.update();

            if (gamepad1.left_bumper) {

                if (limelight.angleToGoalBLUE() > -9) {
                    rx = limelight.angleToGoalBLUE();
                    commandDegrees = Math.toDegrees(drive.localizer.getPose().heading.toDouble());
                } else {
                    commandDegrees = -45;
                    rx = turnController.calculate(Math.toDegrees(drive.localizer.getPose().heading.toDouble()), commandDegrees);
                }
            } else {
                rx = turnController.calculate(Math.toDegrees(drive.localizer.getPose().heading.toDouble()), commandDegrees);
            }

            telemetry.addData("Shooter left velo", shooter.shootLeft.getVelocity());
            telemetry.addData("Shooter right velo", shooter.shootRight.getVelocity());
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