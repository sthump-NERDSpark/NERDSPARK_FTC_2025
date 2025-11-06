package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.PID;
import org.firstinspires.ftc.teamcode.Util.TimerWait;

public class Shooter {
    public enum ShooterActions {
        Intake,
        IntakeHuman,
        AlignAndAim,
        Shoot,
        AimInPlaceFar,
        AimInPLaceClose,
        ZeroPower,
        Park,
        NoAction
    }
    private ShooterActions currentAction;
    private final MecanumDrive Drive;
    private final boolean alliance_blue;
    private final LimelightManager limelight;

    public final DcMotorEx shootTop;
    public final DcMotorEx shootBottom;
    private final DcMotorEx pivotLeft;
    private final DcMotorEx pivotRight;
    private final AnalogInput potentiometer;
    private final PID controller = new PID(0.015,0.0002,0);

    private final Servo kickLeft;
    private final Servo kickCenter;
    private final Servo kickRight;
    private int servoCounter = 0;
//    private final Servo park;

    private final NormalizedColorSensor sensorLeft;
    private final NormalizedColorSensor sensorCenter;
    private final NormalizedColorSensor sensorRight;

    // FOR SHOOTING
    private double shooterBottomVelocity;
    private double shooterTopVelocity;

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
    public shootOrder shotOrder = null;

    // FOR FINDING BEST SHOT
    private static final double G_ft = 32.174; // gravity in ft/s²
    private static final double INCHES_TO_FEET = 1.0 / 12.0;
    private static final double WHEEL_DIAMETER_INCHES = 2.0; // contact wheel
    private static final double PI = Math.PI;

    public static class Result {
        public double angleDeg;
        public double topMotorDegPerSec;
        public double bottomMotorDegPerSec;
        public double error;
    }

    private static final Vector2d blueGoalPose = new Vector2d(63,-55);
    private static final Vector2d redGoalPose = new Vector2d(63,55);
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();
    private final TimerWait waiter = new TimerWait();

    public Shooter(HardwareMap hardwareMap,MecanumDrive drive,boolean alliance,Telemetry telemetry) {
        this.Drive = drive;
        this.alliance_blue = alliance;
        this.telemetry = telemetry;
        this.limelight = new LimelightManager(hardwareMap);

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
        shootTop.setDirection(DcMotorSimple.Direction.FORWARD);
        shootBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        shootTop.setVelocityPIDFCoefficients(55,0.6,0.9,20);
        shootBottom.setVelocityPIDFCoefficients(55,0.6,0.9,20);

        kickLeft = hardwareMap.get(Servo.class, "leftKick");
        kickCenter = hardwareMap.get(Servo.class, "centerKick");
        kickRight = hardwareMap.get(Servo.class, "rightKick");
//        park = hardwareMap.get(Servo.class, "park");
        // TODO
//        park.setPosition(0);

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
            case Shoot: Shoot(); break;
            case Intake: Intake(); break;
            case IntakeHuman: IntakeHuman(); break;
            case AlignAndAim: alignAndAim(); break;
            case AimInPlaceFar: AimInPlaceFar(); break;
            case AimInPLaceClose: AimInPlaceClose(); break;
            case ZeroPower: ZeroPower(); break;
            case Park: Park(); break;
        }
    }

    public double getPotPosition() {
        double currVolts = potentiometer.getVoltage();
        double position = ((270*currVolts+445.5)-Math.sqrt(Math.pow(270*currVolts+445.5, 2) + 4*currVolts*(36450*currVolts-120285)))/(2*currVolts);
        return position - 83.6796;
    }

    /**
     * Finds the best shooter angle and motor speeds for a given distance and fixed height.
     * @param distanceInches  distance from shooter to target (inches)
     * @param heightInches    fixed height difference (targetY - shooterY)
     * @param minAngleDeg     minimum shooter angle (deg)
     * @param horizontalAngleDeg mechanical angle where shooter is level with the floor (deg)
     * @param maxAngleDeg     maximum shooter angle (deg)
     * @param minSpeedFtPerSec  minimum launch speed to test (ft/s)
     * @param maxSpeedFtPerSec  maximum launch speed to test (ft/s)
     * @param angleStepDeg    step size for angle (deg)
     * @param speedStepFtPerSec step size for speed (ft/s)
     * @param topToBottomRatio ratio of top to bottom motor speed (>1 = topspin)
     */
    private static Result findBestShot(double distanceInches, double heightInches,
                                       int minAngleDeg, double maxAngleDeg, double horizontalAngleDeg,
                                       double minSpeedFtPerSec, double maxSpeedFtPerSec,
                                       int angleStepDeg, double speedStepFtPerSec,
                                       double topToBottomRatio) {

        double dx = distanceInches * INCHES_TO_FEET;
        double dy = heightInches * INCHES_TO_FEET;
        double bestError = Double.MAX_VALUE;
        Result best = new Result();

        for (int angle = minAngleDeg; angle <= maxAngleDeg; angle += angleStepDeg) {
            double theta = Math.toRadians(angle - horizontalAngleDeg);

            for (double v = minSpeedFtPerSec; v <= maxSpeedFtPerSec; v += speedStepFtPerSec) {
                // projectile motion equation: y = x*tan(a) - (g*x²)/(2*v²*cos²(a))
                double yPred = dx * Math.tan(theta)
                        - (G_ft * dx * dx) / (2 * v * v * Math.pow(Math.cos(theta), 2));

                double error = Math.abs(yPred - dy);

                if (error < bestError) {
                    bestError = error;
                    best.angleDeg = angle;
                    best.error = error;

                    // Convert ball linear speed to wheel angular speed
                    double wheelRadiusFeet = (WHEEL_DIAMETER_INCHES / 12.0) / 2.0;
                    double wheelAngularVelocityRadPerSec = v / wheelRadiusFeet;
                    double wheelAngularVelocityDegPerSec = Math.toDegrees(wheelAngularVelocityRadPerSec);

                    // Split speeds for spin
                    double avgFactor = (topToBottomRatio + 1.0) / 2.0;
                    best.topMotorDegPerSec = wheelAngularVelocityDegPerSec * topToBottomRatio / avgFactor;
                    best.bottomMotorDegPerSec = wheelAngularVelocityDegPerSec / avgFactor;
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
    private void AimAndSpinUp() {
        if (limelight.getDistance(alliance_blue) > 0) {
            Result r = findBestShot(limelight.getDistance(alliance_blue), 15,
                    0, 110, 43, 500, 1200,
                    5, 10, 1.2);

            shooterTopVelocity = r.topMotorDegPerSec;
            shooterBottomVelocity = r.bottomMotorDegPerSec;

            double command = controller.calculate(r.angleDeg, getPotPosition());
            pivotLeft.setPower(command);
            pivotRight.setPower(command);

            if (Math.abs(r.angleDeg - getPotPosition()) <= 5) {
                shootTop.setVelocity(shooterTopVelocity, AngleUnit.DEGREES);
                shootBottom.setVelocity(shooterBottomVelocity, AngleUnit.DEGREES);
            }
        } else {
            telemetry.addLine("To Close To Read Tag!");
            telemetry.update();
        }
    }

    private void AimInPlaceFar() {
        shooterBottomVelocity = 1200;
        shooterTopVelocity = 1600;

        double command = controller.calculatePosition(110, getPotPosition());
        pivotLeft.setPower(command);
        pivotRight.setPower(command);

        if (Math.abs(110 - getPotPosition()) <= 5) {
            telemetry.addLine("Spinning up wheels");
            shootTop.setVelocity(shooterTopVelocity);
            shootBottom.setVelocity(shooterBottomVelocity);
        }
    }
    private void AimInPlaceClose() {
        shooterBottomVelocity = 1000;
        shooterTopVelocity = 1300;

        double command = controller.calculatePosition(110, getPotPosition());
        pivotLeft.setPower(command);
        pivotRight.setPower(command);

        if (Math.abs(110 - getPotPosition()) <= 5) {
            telemetry.addLine("Spinning up wheels");
            shootTop.setVelocity(shooterTopVelocity);
            shootBottom.setVelocity(shooterBottomVelocity);
        }
    }

    // Build the servo order dynamically based on order
    private Servo[] getServoOrder(shootOrder order) {
        if (order != null) {
            switch (order) {
                case LEFT: return new Servo[]{kickLeft, kickCenter, kickRight};
                case CENTER_LEFT: return new Servo[]{kickCenter, kickLeft, kickRight};
                case CENTER_LAST: return new Servo[]{kickLeft, kickRight, kickCenter};
                case CENTER_RIGHT: return new Servo[]{kickCenter, kickRight, kickLeft};
                case RIGHT: return new Servo[]{kickRight, kickLeft, kickCenter};
            }
        }
        return new Servo[]{kickLeft, kickCenter, kickRight};
    }

    private boolean motorsAtVelocity(double targetTop, double targetBottom) {
        double VELOCITY_TOLERANCE = 50;

        double top = shootTop.getVelocity();
        double bottom = shootBottom.getVelocity();
        return Math.abs(top - (targetTop)) < VELOCITY_TOLERANCE ||
                Math.abs(bottom - targetBottom) < VELOCITY_TOLERANCE;
    }

    private void Shoot() {
        telemetry.addLine("Started shooting");
        telemetry.update();
        if (shotOrder == null) {
            getServoOrder();
        }
        Servo[] sequence = getServoOrder(shotOrder);
        telemetry.addData("Servo order: ", sequence);

        // ResetAndWait until motors are at target velocity
        if (motorsAtVelocity(shooterTopVelocity, shooterBottomVelocity)) {
            Servo servo = sequence[servoCounter];
            // Move the current servo
            servo.setPosition(0.85);
            telemetry.addLine("Moved servo");
            waiter.startWait(750);
            if (waiter.isDone()) {
                servo.setPosition(0.65);
                servoCounter++;
            }
            telemetry.update();
        }
        if (servoCounter > 2) {
            servoCounter = 0;
            currentAction = ShooterActions.Intake;
        }
    }

    /**
     * This action shoots the balls in a specified order
     */
    private void getServoOrder() {
        if (shotOrder == null) {
            greenShot first = limelight.getOrder(alliance_blue);
            double[] hues = {
                    JavaUtil.colorToHue(sensorLeft.getNormalizedColors().toColor()),
                    JavaUtil.colorToHue(sensorCenter.getNormalizedColors().toColor()),
                    JavaUtil.colorToHue(sensorRight.getNormalizedColors().toColor())
            };
            for (int i = 0; i < hues.length; i++) {
                if (hues[i] <= 213) {
                    switch (i) {
                        // Green ball is in left
                        case 0:
                            telemetry.addLine("Green ball left");
                            switch (first) {
                                case FIRST:
                                    shotOrder = shootOrder.LEFT; break;
                                case SECOND:
                                    shotOrder = shootOrder.CENTER_LEFT; break;
                                case THIRD:
                                    shotOrder = shootOrder.RIGHT; break;
                            }
                            // Green ball is in center
                        case 1:
                            telemetry.addLine("Green ball center");
                            switch (first) {
                                case FIRST:
                                    shotOrder = shootOrder.CENTER_LEFT; break;
                                case SECOND:
                                    shotOrder = shootOrder.LEFT; break;
                                case THIRD:
                                    shotOrder = shootOrder.CENTER_LAST; break;
                            }
                            // Green ball is in right
                        case 2:
                            telemetry.addLine("Green ball right");
                            switch (first) {
                                case FIRST:
                                    shotOrder = shootOrder.RIGHT; break;
                                case SECOND:
                                    shotOrder = shootOrder.CENTER_RIGHT; break;
                                case THIRD:
                                    shotOrder = shootOrder.LEFT; break;
                            }
                    }
                }
            }
        }
    }

    /**
     * This action aligns the robot to the goal, aims the shooter, and spins up the motors
     * Can take currPose from limelight or odo
     * Can be changed to use limelight distance to tag
     */
    private void alignAndAim() {
        this.Drive.localizer.update();
        //Target X - actual X, target Y - actual Y
        double heading = Math.atan2((this.alliance_blue? blueGoalPose.x : redGoalPose.x) - limelight.getBotPose(Drive, telemetry).position.x,
                (this.alliance_blue? blueGoalPose.y : redGoalPose.y) - limelight.getBotPose(Drive, telemetry).position.y);
        this.Drive.new TurnAction(new TimeTurn(this.Drive.localizer.getPose(), heading, this.Drive.defaultTurnConstraints));
        AimAndSpinUp();
    }

    private void Intake() {
        double command = controller.calculatePosition(-7, getPotPosition());
        pivotLeft.setPower(command);
        pivotRight.setPower(command);

        shootTop.setPower(-0.75);
        shootBottom.setPower(0);
    }
    private void IntakeHuman() {
        double command = controller.calculatePosition(105, getPotPosition());
        pivotLeft.setPower(command);
        pivotRight.setPower(command);

        shootTop.setPower(-0.25);
        shootBottom.setPower(-0.25);
    }

    private void ZeroPower() {
        shootTop.setVelocity(0);
        shootBottom.setVelocity(0);
        pivotLeft.setPower(0);
        pivotRight.setPower(0);
    }

    private void Park() {
        shootTop.setVelocity(0);
        shootBottom.setVelocity(0);
        pivotLeft.setPower(0);
        pivotRight.setPower(0);
//        waiter.startWait(500);
//        if (waiter.isDone()) {
//          park.setPosition(0.1);
//        }
    }
}