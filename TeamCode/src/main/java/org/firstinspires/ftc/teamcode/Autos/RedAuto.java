package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LimelightManager;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Util.PID;

@Autonomous(name = "Red Auton", preselectTeleOp = "Red Teleop for Mecanum Drive")
public class RedAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        double commandDegrees = 0;
        PID turnController = new PID(0.022, 0, 0.0000005);
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");
        LimelightManager limelight = new LimelightManager(hardwareMap, telemetry, true);
        Shooter shooter = new Shooter(hardwareMap, drive, true, telemetry, limelight);
        shooter.setAction(Shooter.ShooterActions.NoAction);
        double x;
        double y = 0;
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (timer.milliseconds() > 400) {
                x = 0;
            } else {
                x = 1;
            }
            double rx = turnController.calculate(Math.toDegrees(drive.localizer.getPose().heading.toDouble()), commandDegrees);
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