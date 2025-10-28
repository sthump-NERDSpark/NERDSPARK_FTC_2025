package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.PID;

@Config
public class Shooter {
    public enum ShooterActions {
        Intake,
        AlignAndAim,
        Shoot,
        AimAndSpinUp,
        AimInPlace,
        Zero
    }
    private ShooterActions currentAction = ShooterActions.Zero;
    private final MecanumDrive Drive;
    private final boolean alliance_blue;

    public final DcMotorEx shootTop;
    public final DcMotorEx shootBottom;
    public final DcMotorEx pivotLeft;
    public final DcMotorEx pivotRight;
    public final AnalogInput potentiometer;
    public static double angleCommand = 60;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    private final Servo kickLeft;
    private final Servo kickCenter;
    private final Servo kickRight;

    private final NormalizedColorSensor sensorLeft;
    private final NormalizedColorSensor sensorCenter;
    private final NormalizedColorSensor sensorRight;

    // FOR SHOOTING
    private double shooterVelocity;

    // FOR CHOOSING BALL ORDER
    public enum shootOrder {
        LEFT,
        CENTER_LEFT,
        CENTER_LAST,
        CENTER_RIGHT,
        RIGHT
    }
    public enum greenShot {
        FIRST,
        SECOND,
        THIRD
    }

    // FOR FINDING BEST SHOT
    // Gravity constant (m/s^2)
    //private static final double G_m = 9.81;
    // Gravity constant (ft/s^2)
    private static final double G_ft = 32.174;

    private static class Result {
        public int angleDeg;
        public double speed;
        public double error;
    }

    private static final Vector2d blueGoalPose = new Vector2d(63,-55);
    private static final Vector2d redGoalPose = new Vector2d(63,55);

    /**
     * Remember to STOP_AND_RESET the pivot encoders in auto init but not in teleop due to the possibility
     * that the starting position may not be exact
     */
    public Shooter(HardwareMap hardwareMap,MecanumDrive drive,boolean alliance) {
        this.Drive = drive;
        this.alliance_blue = alliance;

        shootTop = hardwareMap.get(DcMotorEx.class, "shootTop");
        shootBottom = hardwareMap.get(DcMotorEx.class, "shootBottom");
        pivotLeft = hardwareMap.get(DcMotorEx.class, "leftPivot");
        pivotRight = hardwareMap.get(DcMotorEx.class, "rightPivot");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        shootTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotRight.setDirection(DcMotorSimple.Direction.FORWARD);
        pivotLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shootTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootTop.setVelocityPIDFCoefficients(55,0.6,0.9,20);
        shootBottom.setVelocityPIDFCoefficients(55,0.6,0.9,20);

        kickLeft = hardwareMap.get(Servo.class, "leftKick");
        kickCenter = hardwareMap.get(Servo.class, "centerKick");
        kickRight = hardwareMap.get(Servo.class, "rightKick");

        // Uncomment if needed
        kickLeft.setDirection(Servo.Direction.FORWARD);
        kickCenter.setDirection(Servo.Direction.FORWARD);
        kickRight.setDirection(Servo.Direction.REVERSE);

        sensorLeft = hardwareMap.get(NormalizedColorSensor.class, "leftColor");
        sensorCenter = hardwareMap.get(NormalizedColorSensor.class, "centerColor");
        sensorRight = hardwareMap.get(NormalizedColorSensor.class, "rightColor");
    }

    public void setAction(ShooterActions action) {
        this.currentAction = action;
    }

    public void updateAction() {
        switch (currentAction) {
            case Shoot: shoot();
            case Intake: Intake();
            case AlignAndAim: alignAndAim();
            case AimAndSpinUp: AimAndSpinUp();
            case AimInPlace: AimInPlace();
            case Zero: Zero();
        }
    }

    public double getPotPosition() {
        double currVolts = potentiometer.getVoltage();
        double position = ((270*currVolts+445.5)-Math.sqrt(Math.pow(270*currVolts+445.5, 2) + 4*currVolts*(36450*currVolts-120285)))/(2*currVolts);
        return position - 24.7328;
    }

    /**
     * MAKE SURE ALL UNITS ARE IN FT, FT/S^2
     * Currently runs on robot pose and goal pose but can be changed to use limelight distance plus some
     */
    private static Result findBestShot(
            double xStart, double yStart,
            double xEnd, double yEnd, // Can replace these with limelight distance to tag plus some
            // double ll_dx, double ll_dy
            int minAngle, double maxAngle, double angleHorizontal,
            double minSpeed, double maxSpeed,
            int angleStep, double speedStep) {

        double dx = xEnd - xStart; // Can replace with limelight distance to tag plus some
        // double dx = ll_dx + 5; // Need to find offset
        double dy = yEnd - yStart; // Can replace with limelight distance to tag plus some
        // double dy = ll_dy + 5 // Need to find offset

        double bestError = Double.MAX_VALUE;
        Result best = new Result();

        for (int angle = minAngle; angle <= maxAngle; angle += angleStep) {
            // Convert to radians relative to horizontal
            double theta = Math.toRadians(angle - angleHorizontal);

            for (double v = minSpeed; v <= maxSpeed; v += speedStep) {
                double yPred = dx * Math.tan(theta)
                        - (G_ft * dx * dx) / (2 * v * v * Math.cos(theta) * Math.cos(theta));

                double error = Math.abs(yPred - dy);

                if (error < bestError) {
                    bestError = error;
                    best.angleDeg = angle;
                    best.error = error;

                    // Convert ball linear speed to wheel angular speed
                    double radius = 0.0508 / 2.0;
                    double omegaRadPerSec = v / radius;
                    best.speed = omegaRadPerSec * (180.0 / Math.PI);
                }
            }
        }

        return best;
    }

    /**
     * This action aims the shooter and spins up the motors
     * Uses odo position
     * Can be changed to use limelight distance to tag
     */
    public void AimAndSpinUp() { // @NonNull LimelightManager ll,
        // Vector2d vect = ll.getDistance();
        this.Drive.localizer.update();

        Result r = findBestShot(this.Drive.localizer.getPose().position.x + 0,
                this.Drive.localizer.getPose().position.y + 0,
                this.alliance_blue? blueGoalPose.x : redGoalPose.x, this.alliance_blue? blueGoalPose.y : redGoalPose.y,
                // vect.x, vect.y,
                1, 75, 40, 375, 1400,
                5, 25);

        shooterVelocity = r.speed;

//        double command = controller.calculate(70, getPotPosition());
//        pivotLeft.setPower(command);
//        pivotRight.setPower(command);

        shootTop.setDirection(DcMotorSimple.Direction.FORWARD);
        shootBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        shootTop.setVelocity(shooterVelocity, AngleUnit.DEGREES);
        shootBottom.setVelocity(shooterVelocity, AngleUnit.DEGREES);

//      packet.put("Height error: ", r.error);
    }

    public void AimInPlace() {
        // TODO: Tune velocity and position
        shooterVelocity = 1000;

        PID controller = new PID(kP,kI,kD);
        double command = controller.calculatePosition(angleCommand, getPotPosition());
        pivotLeft.setPower(command);
        pivotRight.setPower(command);

        if (Math.abs(angleCommand - pivotLeft.getCurrentPosition()) <= 5) {
            shootTop.setDirection(DcMotorSimple.Direction.FORWARD);
            shootBottom.setDirection(DcMotorSimple.Direction.FORWARD);
            shootTop.setVelocity(shooterVelocity);
            shootBottom.setVelocity(shooterVelocity);
        }
    }

    public class Shoot {
        private static final long TIMEOUT_MS = 5000;

        // Build the servo order dynamically based on order
        private Servo[] getServoOrder(shootOrder order) {
            switch (order) {
                case LEFT: return new Servo[]{kickLeft, kickCenter, kickRight};
                case CENTER_LEFT: return new Servo[]{kickCenter, kickLeft, kickRight};
                case CENTER_LAST: return new Servo[]{kickLeft, kickRight, kickCenter};
                case CENTER_RIGHT: return new Servo[]{kickCenter, kickRight, kickLeft};
                case RIGHT: return new Servo[]{kickRight, kickLeft, kickCenter};
            }
            return new Servo[]{kickLeft, kickCenter, kickRight};
        }

        private boolean waitUntilVelocityReached(double target) {
            long start = System.nanoTime();
            long timeoutNs = TIMEOUT_MS * 1_000_000L;

            while (true) {
                if (motorsAtVelocity(target)) return true;
                if (System.nanoTime() - start > timeoutNs) return false;
            }
        }

        private boolean motorsAtVelocity(double target) {
            double VELOCITY_TOLERANCE = 50;

            double left = shootTop.getVelocity();
            double right = shootBottom.getVelocity();
            return Math.abs(left - target) < VELOCITY_TOLERANCE &&
                    Math.abs(right - target) < VELOCITY_TOLERANCE;
        }

        public Shoot(shootOrder order) {
            Servo[] sequence = getServoOrder(order);
//            packet.put("Servo order:", sequence);

            for (Servo servo : sequence) {
                // Wait until motors are at target velocity
                while (!waitUntilVelocityReached(shooterVelocity)) {
//                    packet.put("Timeout waiting for motors before moving servo.", true);
                    servo.setPosition(0.65);
                }

                // Move the current servo
                // TODO: Find position
                servo.setPosition(0.85);
//                packet.put("Moved servo to position ", true);
                // TODO: Find position
                servo.setPosition(0.65);
            }
        }
    }

    /**
     * This action shoots the balls in a specified order
     * Has code for limelight
     */
    public void shoot() { // @NonNull LimelightManager ll, boolean alliance_blue
        greenShot first = greenShot.FIRST; // ll.getOrder(alliance_blue);
        double[] hues = {
                JavaUtil.colorToHue(sensorLeft.getNormalizedColors().toColor()),
                JavaUtil.colorToHue(sensorCenter.getNormalizedColors().toColor()),
                JavaUtil.colorToHue(sensorRight.getNormalizedColors().toColor())
        };
        for (int i = 0; i < hues.length; i++) {
            if (hues[i] >= 79f && hues[i] <= 139f) {
                switch (i) {
                    // Green ball is in left
                    case 0:
                        switch (first) {
                            case FIRST:
                                new Shoot(shootOrder.LEFT);
                            case SECOND:
                                new Shoot(shootOrder.CENTER_LEFT);
                            case THIRD:
                                new Shoot(shootOrder.RIGHT);
                        }
                    // Green ball is in center
                    case 1:
                        switch (first) {
                            case FIRST:
                                new Shoot(shootOrder.CENTER_LEFT);
                            case SECOND:
                                new Shoot(shootOrder.LEFT);
                            case THIRD:
                                new Shoot(shootOrder.CENTER_LAST);
                        }
                    // Green ball is in right
                    case 2:
                        switch (first) {
                            case FIRST:
                                new Shoot(shootOrder.RIGHT);
                            case SECOND:
                                new Shoot(shootOrder.CENTER_RIGHT);
                            case THIRD:
                                new Shoot(shootOrder.LEFT);
                        }
                }
            }
        }
        new Shoot(shootOrder.LEFT);
    }

    /**
     * This action aligns the robot to the goal, aims the shooter, and spins up the motors
     * Can take currPose from limelight or odo
     * Can be changed to use limelight distance to tag
     */
    public void alignAndAim() {
        this.Drive.localizer.update();
        //Target X - actual X, target Y - actual Y
        double heading = Math.atan2((this.alliance_blue? blueGoalPose.x : redGoalPose.x) - this.Drive.localizer.getPose().position.x,
                (this.alliance_blue? blueGoalPose.y : redGoalPose.y) - this.Drive.localizer.getPose().position.y);
        this.Drive.new TurnAction(new TimeTurn(this.Drive.localizer.getPose(), heading, this.Drive.defaultTurnConstraints));
        AimAndSpinUp();
    }

    public void Intake() {
//        double command = controller.calculate(1, getPotPosition());
//        pivotLeft.setPower(command);
//        pivotRight.setPower(command);

        shootTop.setPower(-0.5);
        shootBottom.setPower(0);
    }

    private void Zero() {
        shootTop.setVelocity(0);
        shootBottom.setVelocity(0);
//        double command = controller.calculate(2, getPotPosition());
//        pivotLeft.setPower(command);
//        pivotRight.setPower(command);
        kickLeft.setPosition(0.65);
        kickCenter.setPosition(0.65);
        kickRight.setPosition(0.65);
    }
}