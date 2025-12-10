package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NewLimelightManager;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.NewShooter;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Util.PID;

@TeleOp(name = "Testing Blue Teleop for Mecanum Drive", group = "comp")
public class NewBlueTeleopMecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Change new Pose2d to match where you start out of auto
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose, false);
        double commandDegrees = PoseStorage.currentPose.heading.toDouble();
        double rx;
        PID turnController = new PID(0.02, 0, 0.0000001);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");
        dashboard.startCameraStream(ll, 0);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        NewLimelightManager limelight = new NewLimelightManager(hardwareMap, telemetry, true, drive);
        NewShooter shooter = new NewShooter(hardwareMap, drive,true, telemetry, limelight);
        shooter.setAction(NewShooter.ShooterActions.NoAction);

        drive.localizer.setPose(PoseStorage.currentPose);

        telemetry.clear();

        limelight.setPipeline(2);

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

            // Auto turn with d-pad
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
            // Human intake, raise shooter and intake slow
            if (gamepad1.b) {
                shooter.setAction(NewShooter.ShooterActions.IntakeHuman);
            }
            // Shoot balls
            if (gamepad2.x) {
                shooter.setAction(NewShooter.ShooterActions.Shoot);
            }
            // Ground intake
            if (gamepad1.a) {
                shooter.setAction(NewShooter.ShooterActions.Intake);
            }
            // Power off
            if (gamepad2.right_bumper || gamepad1.right_bumper) {
                shooter.setAction(NewShooter.ShooterActions.ZeroPower);
            }
            // Angle far shooting manual
            if (gamepad2.y) {
                commandDegrees = -60;
            }
            // Angle close shooting manual
            if (gamepad2.left_trigger > 0.25) {
                commandDegrees = -45;
            }
            // Park in endgame
            if (gamepad2.left_bumper) {
                shooter.setAction(NewShooter.ShooterActions.Park);
                commandDegrees = 180;
            }

//            shooter.getPose();

            drive.localizer.update();

            // Auto turn, if left bumper is pressed raise shooter and aligns angle to goal, else normal auto turn
            if (gamepad1.left_bumper) {
//                shooter.setAction(Shooter.ShooterActions.AimInPLaceClose);
//                if (limelight.angleToGoalBLUE() > -9) {
//                    rx = limelight.angleToGoalBLUE();
//                    commandDegrees = Math.toDegrees(drive.localizer.getPose().heading.toDouble());
//                    commandDegrees = limelight.angleToGoalBLUE();
//                } else {
//                    commandDegrees = -45;
//                    rx = turnController.calculate(Math.toDegrees(drive.localizer.getPose().heading.toDouble()), commandDegrees);
//                }
                commandDegrees = limelight.angleToGoalBLUE();
            }

            rx = turnController.calculate(Math.toDegrees(drive.localizer.getPose().heading.toDouble()), commandDegrees);

            shooter.updateAction();
//            limelight.angleToGoalBLUE();

            telemetry.addData("Commanded Degrees: ", commandDegrees);
            telemetry.addData("Current Degrees: ", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

            telemetry.addData("Motor Command: ", rx);

            telemetry.addData("Shooter Top Actual", shooter.shootTop.getVelocity());
            telemetry.addData("Shooter Bottom Actual", shooter.shootBottom.getVelocity());
            telemetry.update();

            // ********************* Driving Math ***** Do NOT Change *********************
            double botHeading = drive.localizer.getPose().heading.toDouble();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), rx));
        }
    }
}