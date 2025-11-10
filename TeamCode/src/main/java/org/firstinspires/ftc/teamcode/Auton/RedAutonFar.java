package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.LimelightManager;

@Autonomous(name = "Red Auton Far", group = "Comp")
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
            Math.toRadians(-90)   // heading in radians (facing +X, front wall)
    );

    private static final double FIRST_FORWARD_DIST = 8.0; // forward to first shooting line

    // ---- MIRRORED VALUES FOR RED ----
    private static final double FIRST_CORNER_Y = -40.0;   // mirrored corner (negative Y)
    private static final double SECOND_ARTIFACT_X = 30.0; // same forward distance
    private static final double FINAL_SHOT_Y = 12.0;      // mirrored strafe (positive Y now)

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
        // On red, turn CW to face the right-hand corner artifacts.
        pose = drive.localizer.getPose();

        Action turnCW90 = drive.actionBuilder(pose)
                .turn(Math.toRadians(-90))
                .build();

        Actions.runBlocking(turnCW90);
        pose = drive.localizer.getPose();

        // While moving to corner, put shooter in ZeroPower.
        shooter.setAction(Shooter.ShooterActions.ZeroPower);
        shooter.updateAction();

        // Drive toward the red-side corner artifacts (negative Y).
        Action toFirstCorner = drive.actionBuilder(pose)
                .lineToY(FIRST_CORNER_Y)
                .build();

        Actions.runBlocking(toFirstCorner);
        pose = drive.localizer.getPose();

        // Within ~3" of target: Intake
        shooter.setAction(Shooter.ShooterActions.Intake);
        shooter.updateAction();

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
        Action backToFirstShotLane = drive.actionBuilder(pose)
                .lineToY(0.0)
                .lineToX(FIRST_FORWARD_DIST)
                .build();

        Actions.runBlocking(backToFirstShotLane);
        pose = drive.localizer.getPose();

        // Second shot
        aimAndShootClose(shooter, 1.5, 2.5);

        // -------------------- STEP 5: DRIVE TO SECOND ARTIFACT SET (HEADING STAYS SAME) --------------------
        shooter.setAction(Shooter.ShooterActions.ZeroPower);
        shooter.updateAction();

        pose = drive.localizer.getPose();

        Action toSecondArtifacts = drive.actionBuilder(pose)
                .lineToX(SECOND_ARTIFACT_X)
                .build();

        Actions.runBlocking(toSecondArtifacts);
        pose = drive.localizer.getPose();

        // Within ~3" of target: Intake
        shooter.setAction(Shooter.ShooterActions.Intake);
        shooter.updateAction();

        // -------------------- STEP 6: PREP THIRD SHOT AND STRAFE LEFT TO FINAL SHOOT --------------------
        // Red side mirrors the Blue's "strafe right" → here we strafe left (positive Y)
        shooter.setAction(Shooter.ShooterActions.AimInPLaceClose);
        runShooterForTime(shooter, 0.5);

        pose = drive.localizer.getPose();

        Action strafeToFinalShot = drive.actionBuilder(pose)
                .lineToY(FINAL_SHOT_Y)
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
}