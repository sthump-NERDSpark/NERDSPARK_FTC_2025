package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.LimelightManager;
import org.firstinspires.ftc.teamcode.NewLimelightManager;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Shooter;
@Config
@Autonomous(name = "BlueAutonFar", group = "Comp")
public class BlueAutonFar extends LinearOpMode {

    // ---------------- FIELD / PATH CONSTANTS ----------------
    // Coordinate conventions:
    //  - X axis: forward toward the front wall
    //  - Y axis: left from the blue alliance perspective
    //  - Heading 0 rad: facing the front wall
    //
    // Robot starts flat on the back wall, shooter facing front wall.

    public static double firstpointAngle = -65;
    public static double firstpointX = 0;
    public static double firstpointY = -10;
    public static double secondpointX = 0;
    public static double secondpointY = -20.0;
    public static double secondpointAngle = -90;
    public static double thirdpointX = 36.0;
    public static double thirdpointY = 20.0;
    public static double thirdpointAngle = 90;
    public static double fourthpointX = 5.0;
    public static double fourthpointY = 30.0;
    public static double fourthpointAngle = 95;

   private static final double INTAKE_DISTANCE_INCHES = 3.0;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean allianceBlue = true;   // This is BLUE side

        // ---------- INIT SUBSYSTEMS ----------
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        LimelightManager limelight = new LimelightManager(hardwareMap, telemetry, allianceBlue);
        Shooter shooter = new Shooter(hardwareMap, drive, allianceBlue, telemetry, limelight);

        // Optionally set a dedicated pipeline for auto aiming
        limelight.setPipeline(0);

        drive.localizer.setPose(startPose);

        telemetry.addLine("BlueAutonFar: Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        shooter.setAction(Shooter.ShooterActions.AimInPLaceClose);
        shooter.updateAction();

        // -------------------- STEP 1: DRIVE FORWARD 8" TO FIRST SHOT --------------------
        Pose2d pose = drive.localizer.getPose();

        Action forwardToFirstShot = drive.actionBuilder(pose)
                .splineTo(new Vector2d(firstpointX,firstpointY), Math.toRadians(firstpointAngle))
                .build();

        Actions.runBlocking(forwardToFirstShot);

        // -------------------- STEP 2: FIRST SHOT (AimInPlaceClose + Shoot) --------------------
        aimAndShootClose(shooter, 3, 3);



        // While moving to corner, put shooter in ZeroPower.
        shooter.setAction(Shooter.ShooterActions.ZeroPower);
        shooter.updateAction();

        // Drive toward the blue-side corner artifacts (positive Y).
        //pose = drive.localizer.getPose();
        Action toFirstCorner = drive.actionBuilder(pose)
                .splineTo(new Vector2d(secondpointX, secondpointY), Math.toRadians(secondpointAngle))
                .build();

        Actions.runBlocking(toFirstCorner);
//
//        pose = drive.localizer.getPose();
//        // Explicit "within 3 inches" check before switching to INTAKE
//        if (isWithinDistance(pose, secondpointX, secondpointY, INTAKE_DISTANCE_INCHES)) {
//            shooter.setAction(Shooter.ShooterActions.Intake);
//            shooter.updateAction();
//        }
//
//        // -------------------- STEP 4: RETURN TO ORIGINAL SHOT POSE --------------------
//        shooter.setAction(Shooter.ShooterActions.AimInPLaceClose);
//        shooter.updateAction();
//
//        // Drive back to the same line as first shot (y = 0).
//        pose = drive.localizer.getPose();
//        Action backToFirstShotLane = drive.actionBuilder(pose)
//                .splineTo(new Vector2d(firstpointX, firstpointY), Math.toRadians(firstpointAngle))
//                .build();
//
//        Actions.runBlocking(backToFirstShotLane);
//
//        // Second shot
//        aimAndShootClose(shooter, 1, 10);
//
//        // -------------------- STEP 5: DRIVE TO SECOND ARTIFACT SET (HEADING STAYS SAME) --------------------
//        shooter.setAction(Shooter.ShooterActions.ZeroPower);
//        shooter.updateAction();
//
//        pose = drive.localizer.getPose();
//        Action toSecondArtifacts = drive.actionBuilder(pose)
//                .splineTo(new Vector2d(thirdpointX,thirdpointY),Math.toRadians(thirdpointAngle))
//                .build();
//
//        Actions.runBlocking(toSecondArtifacts);
//        pose = drive.localizer.getPose();
//
//        // Explicit “within 3 inches” check before switching to INTAKE
//        if (isWithinDistance(pose, thirdpointX, thirdpointY, INTAKE_DISTANCE_INCHES)) {
//            shooter.setAction(Shooter.ShooterActions.Intake);
//            shooter.updateAction();
//        }
//

        // -------------------- STEP 7: FINISH SAFE --------------------
        shooter.setAction(Shooter.ShooterActions.ZeroPower);
        shooter.updateAction();

        telemetry.addLine("BlueAutonFar complete.");
        telemetry.update();

        PoseStorage.currentPose = drive.localizer.getPose();
    }

    /**
     * Helper to aim & shoot using Limelight.
     * AimInPLaceClose:
     *  - uses Limelight Ty to set shooter velocities & pivot angle
     * Shoot:
     *  - uses motorsAtVelocity(...) to gate servo firing.
     */
    private void aimAndShootClose(Shooter shooter, double aimTimeSec, double shootTimeSec) {
        ElapsedTime timer = new ElapsedTime();

        shooter.setAction(Shooter.ShooterActions.AimInPLaceClose);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < aimTimeSec) {
            shooter.updateAction();
            telemetry.addData("aim timer", timer.seconds());
            telemetry.update();

        }

        shooter.setAction(Shooter.ShooterActions.Shoot);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < shootTimeSec) {
            shooter.updateAction();
            telemetry.addData("shoot timer", timer.seconds());
            telemetry.update();
        }
    }

    /**
     * Helper to run shooter for a fixed time in its current state.
     */
    private void runShooterForTime(Shooter shooter, double timeSec) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < timeSec) {
            shooter.updateAction();
        }
    }

    /**
     * Explicit "within distance" check using Road Runner Pose2d.
     * pose.position.x / pose.position.y are field coordinates in inches.
     */
    private boolean isWithinDistance(Pose2d pose, double targetX, double targetY, double tolInches) {
        double dx = pose.position.x - targetX;
        double dy = pose.position.y - targetY;
        double dist = Math.hypot(dx, dy);
        return dist <= tolInches;
    }
}
