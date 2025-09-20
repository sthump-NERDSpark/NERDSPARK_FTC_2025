package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous
public final class DecodeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .lineToXLinearHeading(5, Math.toRadians(-20))
                        .splineToLinearHeading(new Pose2d(24, 14, Math.toRadians(90)), Math.toRadians(90))
                        .splineTo(new Vector2d(24, 40), Math.toRadians(-90))
                        .splineToLinearHeading(new Pose2d(76, -4, Math.toRadians(-50)), Math.toRadians(-50))
                        .build());
        } else {
            throw new RuntimeException();
        }
    }
}
