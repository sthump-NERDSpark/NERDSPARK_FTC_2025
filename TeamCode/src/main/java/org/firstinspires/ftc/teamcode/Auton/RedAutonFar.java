package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.LimelightManager;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Shooter;

@Autonomous(name = "RedAutonFar", group = "Comp")
public class RedAutonFar extends LinearOpMode {

    // ---------------- FIELD / PATH CONSTANTS ----------------
    // Coordinate conventions:
    //  - X axis: forward toward the front wall
    //  - Y axis: left from the blue alliance perspective (so red is mirrored → negative Y)
    //  - Heading 0 rad: facing the front wall
    //
    // Robot starts flat on the back wall, shooter facing front wall.

    private static final Pose2d START_POSE = new Pose2d(
            0.0,  // x in inches
            0.0,  // y in inches
            0.0   // heading in radians (facing +X, front wall)
    );

    private static final double FIRST_FORWARD_DIST = 8.0; // forward to first shooting line

    // ---- MIRRORED VALUES FOR RED ----
    // Corner artifact location after first 90° CW turn
    private static final double FIRST_CORNER_Y = -40.0;   // negative Y (right side); tune on field
    private static final double SECOND_ARTIFACT_X = 30.0; // forward distance for second pickup; tune
    private static final double FINAL_SHOT_Y = 12.0;      // strafe left from second pickup; tune

    // Distance threshold for “within X inches of target”
    private static final double INTAKE_DISTANCE_INCHES = 3.0;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean allianceBlue = false;   // This is RED side

        // ---------- INIT SUBSYSTEMS ----------
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);
        LimelightManager limelight = new LimelightManager(hardwareMap, telemetry, allianceBlue);
        Shooter shooter = new Shooter(hardwareMap, drive, allianceBlue, telemetry, limelight);

        // Optionally set a dedicated pipeline for auto aiming
        // limelight.setPipeline(1);

        drive.localizer.setPose(START_POSE);

        telemetry.addLine("RedAutonFar: Initialized. Waiting for start...");
        telemetry.update();

        limelight.setPipeline(1);

        waitForStart();
        if (isStopRequested()) return;

        // -------------------- STEP 1: DRIVE FORWARD 8" TO FIRST SHOT --------------------
        Pose2d pose = drive.localizer.getPose();

        Action forwardToFirstShot = drive.actionBuilder(pose)
                .lineToX(pose.position.x + FIRST_FORWARD_DIST)
                .build();

        Actions.runBlocking(forwardToFirstShot);
        pose = drive.localizer.getPose();

        // -------------------- STEP 2: FIRST SHOT (AimInPlaceClose + Shoot) --------------------
        aimAndShootClose(shooter, 1.5, 2.5);

        // -------------------- STEP 3: TURN CLOCKWISE 90°, ZERO SHOOTER, DRIVE TO FIRST CORNER --------------------
        // Turn CW to face the right-hand corner artifacts.
        pose = drive.localizer.getPose();

        Action turnCW90 = drive.actionBuilder(pose)
                .turn(Math.toRadians(-90))
                .build();

        Actions.runBlocking(turnCW90);

        // After turning, x should still be FIRST_FORWARD_DIST (≈ 8")
        pose = drive.localizer.getPose();
        double firstCornerTargetX = FIRST_FORWARD_DIST;
        double firstCornerTargetY = FIRST_CORNER_Y;

        // While moving to corner, put shooter in ZeroPower.
        shooter.setAction(Shooter.ShooterActions.ZeroPower);
        shooter.updateAction();

        // Drive toward the red-side corner artifacts (negative Y).
        Action toFirstCorner = drive.actionBuilder(pose)
                .lineToY(firstCornerTargetY)
                .build();

        Actions.runBlocking(toFirstCorner);
        pose = drive.localizer.getPose();

        // Now explicitly check: are we within 3" of the intended corner target?
        if (isWithinDistance(pose, firstCornerTargetX, firstCornerTargetY, INTAKE_DISTANCE_INCHES)) {
            shooter.setAction(Shooter.ShooterActions.Intake);
            shooter.updateAction();
        }

        // -------------------- STEP 4: RETURN TO ORIGINAL SHOT POSE --------------------
        // Turn back CCW 90° to face front wall again.
        pose = drive.localizer.getPose();
        Action turnCCW90 = drive.actionBuilder(pose)
                .turn(Math.toRadians(90))
                .build();
        Actions.runBlocking(turnCCW90);
        pose = drive.localizer.getPose();

        shooter.setAction(Shooter.ShooterActions.ZeroPower);
        shooter.updateAction();

        // Drive back to the same line as first shot (y = 0).
        double firstShotTargetX = FIRST_FORWARD_DIST;
        double firstShotTargetY = 0.0;

        Action backToFirstShotLane = drive.actionBuilder(pose)
                .lineToY(firstShotTargetY)
                .lineToX(firstShotTargetX)
                .build();

        Actions.runBlocking(backToFirstShotLane);
        pose = drive.localizer.getPose();

        // Second shot
        aimAndShootClose(shooter, 1.5, 2.5);

        // -------------------- STEP 5: DRIVE TO SECOND ARTIFACT SET (HEADING STAYS SAME) --------------------
        shooter.setAction(Shooter.ShooterActions.ZeroPower);
        shooter.updateAction();

        pose = drive.localizer.getPose();
        double secondArtifactTargetX = SECOND_ARTIFACT_X;
        double secondArtifactTargetY = firstShotTargetY;  // should be 0.0

        Action toSecondArtifacts = drive.actionBuilder(pose)
                .lineToX(secondArtifactTargetX)
                .build();

        Actions.runBlocking(toSecondArtifacts);
        pose = drive.localizer.getPose();

        // Explicit “within 3 inches” check before switching to INTAKE
        if (isWithinDistance(pose, secondArtifactTargetX, secondArtifactTargetY, INTAKE_DISTANCE_INCHES)) {
            shooter.setAction(Shooter.ShooterActions.Intake);
            shooter.updateAction();
        }

        // -------------------- STEP 6: PREP THIRD SHOT AND STRAFE LEFT TO FINAL SHOOT --------------------
        // Start aiming at current position
        shooter.setAction(Shooter.ShooterActions.AimInPLaceClose);
        runShooterForTime(shooter, 0.5);

        pose = drive.localizer.getPose();
        double finalShotTargetX = secondArtifactTargetX;   // only changing Y here
        double finalShotTargetY = FINAL_SHOT_Y;

        // Strafe left (positive Y) to final shooting lane
        Action strafeToFinalShot = drive.actionBuilder(pose)
                .lineToY(finalShotTargetY)
                .build();

        Actions.runBlocking(strafeToFinalShot);
        pose = drive.localizer.getPose();

        // Final aim + shoot
        aimAndShootClose(shooter, 1.5, 2.5);

        // -------------------- STEP 7: FINISH SAFE --------------------
        shooter.setAction(Shooter.ShooterActions.ZeroPower);
        shooter.updateAction();

        telemetry.addLine("RedAutonFar complete.");
        telemetry.update();
    }

    /**
     * Helper to aim & shoot using Limelight.
     */
    private void aimAndShootClose(Shooter shooter, double aimTimeSec, double shootTimeSec) {
        ElapsedTime timer = new ElapsedTime();

        shooter.setAction(Shooter.ShooterActions.AimInPLaceClose);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < aimTimeSec) {
            shooter.updateAction();
        }

        shooter.setAction(Shooter.ShooterActions.Shoot);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < shootTimeSec) {
            shooter.updateAction();
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
