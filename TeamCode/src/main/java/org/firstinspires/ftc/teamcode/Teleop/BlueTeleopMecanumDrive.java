package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LimelightManager;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Util.PID;

@TeleOp(name = "Blue Teleop for Mecanum Drive")
public class BlueTeleopMecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Change new Pose2d to match where you start out of auto
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        double commandDegrees = 0;
        PID turnController = new PID(0.02, 0, 0.0000001);
        Shooter shooter = new Shooter(hardwareMap, drive,true, telemetry);
        shooter.setAction(Shooter.ShooterActions.NoAction);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.clear();

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            double y = -gamepad1.right_stick_y;
            double x = -gamepad1.right_stick_x;

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
            if (gamepad1.b) {
                shooter.setAction(Shooter.ShooterActions.IntakeHuman);
            }

            drive.localizer.update();

            telemetry.addData("Commanded Degrees: ", commandDegrees);
            telemetry.addData("Current Degrees: ", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

            double rx = turnController.calculate(Math.toDegrees(drive.localizer.getPose().heading.toDouble()), commandDegrees);
            telemetry.addData("Motor Command: ", rx);

//            if (gamepad1.b) {
//                shooter.setAction(Shooter.ShooterActions.AimAndSpinUp);
//            }
            if (gamepad2.x) {
                shooter.setAction(Shooter.ShooterActions.Shoot);
            }
//            if (gamepad1.a) {
//                shooter.setAction(Shooter.ShooterActions.AlignAndAim);
//            }
            if (gamepad1.a) {
                shooter.setAction(Shooter.ShooterActions.Intake);
            }
            if (gamepad2.b) {
                shooter.setAction(Shooter.ShooterActions.AimInPlaceFar);
            }
            if (gamepad2.right_trigger > 0.25) {
                shooter.setAction(Shooter.ShooterActions.AimInPLaceClose);
            }
            if (gamepad2.right_bumper) {
                shooter.setAction(Shooter.ShooterActions.ZeroPower);
            }
            if (gamepad2.y) {
                commandDegrees = -60;
            }
            if (gamepad2.left_trigger > 0.25) {
                commandDegrees = -45;
            }
            if (gamepad2.left_bumper) {
                shooter.setAction(Shooter.ShooterActions.Park);
            }
            shooter.updateAction();

            telemetry.addData("Target: ", 1200);
            telemetry.addData("Shooter Top Actual", shooter.shootTop.getVelocity());
            telemetry.addData("Shooter Bottom Actual", shooter.shootBottom.getVelocity());
            telemetry.update();

            drive.localizer.update();
            double botHeading = drive.localizer.getPose().heading.toDouble();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), rx));
        }
    }
}