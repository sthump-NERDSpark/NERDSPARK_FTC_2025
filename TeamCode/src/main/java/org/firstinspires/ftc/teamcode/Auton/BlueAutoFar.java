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

@Autonomous(name = "Blue Auton Far", group = "Comp")
public class BlueAutoFar extends LinearOpMode {

    // ---- TUNE THESE POSES / DISTANCES ----
    // Start flat on back wall, shooter facing front wall
    private static final Pose2d START_POSE = new Pose2d(
            0.0, 0.0, Math.toRadians(90)
    );

    // After driving straight forward 8"
    private static final double FORWARD_DISTANCE = 8.0; // inches

    // First shooting location (8" forward)
    private static final Pose2d FIRST_SHOT_POSE = new Pose2d(
            FORWARD_DISTANCE, 0.0, 0.0
    );

    // Corner artifacts after first shot (robot rotated CCW 90°)
    private static final Pose2d FIRST_ARTIFACT_CORNER = new Pose2d(
            FORWARD_DISTANCE, 24.0, Math.toRadians(90) // y=24" is a guess — tune this
    );

    // Second set of artifacts (heading stays the same as coming into it)
    private static final Pose2d SECOND_ARTIFACT_POS = new Pose2d(
            24.0, 0.0, 0.0 // guess: farther downfield in +X; tune this
    );

    // Final shooting location after strafe to the right
    private static final Pose2d FINAL_SHOT_POSE = new Pose2d(
            FORWARD_DISTANCE, -12.0, 0.0 // strafe right 12" from the “lane”
    );

    @Override
    public void runOpMode() throws InterruptedException {
        boolean allianceBlue = true; // this auto is blue side

        // ---- INIT SUBSYSTEMS ----
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);
        LimelightManager limelight = new LimelightManager(hardwareMap, telemetry, allianceBlue);
        Shooter shooter = new Shooter(hardwareMap, drive, allianceBlue, telemetry, limelight);

        // If you have a special pipeline for auto shooting:
        // limelight.setPipeline(1);

        drive.localizer.setPose(START_POSE);

        telemetry.addLine("Blue Left Artifacts Auto - Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ----------------- STEP 1: MOVE FORWARD 8" TO FIRST SHOT -----------------
        Pose2d pose = drive.localizer.getPose();

        Action forward8 = drive.actionBuilder(pose)
                .lineToX(pose.position.x + FORWARD_DISTANCE)
                .build();
        Actions.runBlocking(forward8);

        pose = drive.localizer.getPose();

        // ----------------- STEP 2: AIM & SHOOT (FIRST TIME) -----------------
        aimAndShootClose(shooter, 1.5, 2.5);  // (aimTime, shootTime) in seconds

        // ----------------- STEP 3: ROTATE CCW 90°, ZERO SHOOTER, GO TO FIRST CORNER -----------------
        // Rotate in place +90 degrees
        pose = drive.localizer.getPose();
        Action turnCCW90 = drive.actionBuilder(pose)
                .turn(Math.toRadians(90))
                .build();
        Actions.runBlocking(turnCCW90);

        pose = drive.localizer.getPose();

        // Put shooter in ZeroPower while moving
        shooter.setAction(Shooter.ShooterActions.ZeroPower);
        shooter.updateAction();

        // Drive to artifact corner (you can refine this path with splines if needed)
        Action toFirstCorner = drive.actionBuilder(pose)
                .lineToY(FIRST_ARTIFACT_CORNER.position.y)
                .build();
        Actions.runBlocking(toFirstCorner);

        pose = drive.localizer.getPose();

        // Within ~3" of target: we’re now at the corner → INTAKE
        shooter.setAction(Shooter.ShooterActions.Intake);
        shooter.updateAction();

        // ----------------- STEP 4: PREP SECOND SHOT - RETURN TO FIRST SHOT POSE -----------------
        // Raise shooter & aim close shot again
        shooter.setAction(Shooter.ShooterActions.AimInPLaceClose);

        // Turn back CW 90° first
        pose = drive.localizer.getPose();
        Action turnCW90 = drive.actionBuilder(pose)
                .turn(Math.toRadians(-90))
                .build();
        Actions.runBlocking(turnCW90);

        pose = drive.localizer.getPose();

        // Drive back to original first shot Y position (keeping heading 0)
        Action backToFirstShot = drive.actionBuilder(pose)
                .lineToY(FIRST_SHOT_POSE.position.y)
                .lineToX(FIRST_SHOT_POSE.position.x)
                .build();
        Actions.runBlocking(backToFirstShot);

        pose = drive.localizer.getPose();

        // Let shooter finish aiming, then shoot again
        aimAndShootClose(shooter, 1.5, 2.5);

        // ----------------- STEP 5: SECOND ARTIFACT SET -----------------
        // Drop shooter, move to next set of artifacts without rotating heading
        shooter.setAction(Shooter.ShooterActions.ZeroPower);
        shooter.updateAction();

        pose = drive.localizer.getPose();
        Action toSecondArtifacts = drive.actionBuilder(pose)
                .lineToX(SECOND_ARTIFACT_POS.position.x)
                .build();
        Actions.runBlocking(toSecondArtifacts);

        pose = drive.localizer.getPose();

        // Within ~3" of second artifacts: INTAKE
        shooter.setAction(Shooter.ShooterActions.Intake);
        shooter.updateAction();

        // Once at position, prepare aiming again
        shooter.setAction(Shooter.ShooterActions.AimInPLaceClose);

        // ----------------- STEP 6: MOVE RIGHT TO FINAL SHOOTING LOCATION & SHOOT -----------------
        pose = drive.localizer.getPose();
        // Maintain heading, strafe “right” to FINAL_SHOT_POSE.y
        Action toFinalShoot = drive.actionBuilder(pose)
                .lineToY(FINAL_SHOT_POSE.position.y)
                .build();
        Actions.runBlocking(toFinalShoot);

        pose = drive.localizer.getPose();

        // Final aim & shoot
        aimAndShootClose(shooter, 1.5, 2.5);

        // ----------------- DONE: PARK SHOOTER -----------------
        shooter.setAction(Shooter.ShooterActions.ZeroPower);
        shooter.updateAction();

        telemetry.addLine("Auto Complete");
        telemetry.update();
    }

    /**
     * Helper: run AimInPlaceClose for a bit (to let Limelight set velocities & pivot),
     * then switch to Shoot and let the Shoot state machine fire the servos.
     *
     * @param shooter      your Shooter subsystem
     * @param aimTimeSec   how long to spend aiming/spinning up
     * @param shootTimeSec how long to allow the Shoot state to run
     */
    private void aimAndShootClose(Shooter shooter, double aimTimeSec, double shootTimeSec) {
        ElapsedTime timer = new ElapsedTime();

        // 1) Aim & spin up using Limelight Ty → shooter velocity
        shooter.setAction(Shooter.ShooterActions.AimInPLaceClose);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < aimTimeSec) {
            shooter.updateAction();
        }

        // 2) Switch to Shoot state: internally uses motorsAtVelocity() to fire
        shooter.setAction(Shooter.ShooterActions.Shoot);
        timer.reset();
        while (opModeIsActive() && timer.seconds() < shootTimeSec) {
            shooter.updateAction();
        }

        // After shooting, Shooter.Shoot() sets currentAction back to Intake when done
    }
}
