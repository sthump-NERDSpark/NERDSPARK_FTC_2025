package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.PID;
import org.firstinspires.ftc.teamcode.Util.TimerWait;

@Config
public class Shooter {
    public static double shooterTopConfig = 0;
    public static double shooterBottomConfig = 0;
    public static double shooterAngle = 0;
    public static int velocityTolTimeOut = 3;
    public static double VELOCITY_TOLERANCE = 60;

    public static double inatkePos = 8;
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

    private final Telemetry telemetry;
    private final TimerWait shootWaiter = new TimerWait();
    private boolean waitStarted = false;

    public Shooter(HardwareMap hardwareMap,MecanumDrive drive,boolean alliance,Telemetry telemetry,LimelightManager ll) {
        this.Drive = drive;
        this.alliance_blue = alliance;
        this.telemetry = telemetry;
        this.limelight = ll;


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
        shootTop.setVelocityPIDFCoefficients(55,0.6,0.9,15);
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
//            case AlignAndAim: alignAndAim(); break;
            case AimInPlaceFar: AimAndSpinUpConfig(); break; //AimInPlaceFar(); break;
            case AimInPLaceClose: AimInPlaceClose(); break;
            case ZeroPower: ZeroPower(); break;
            case Park: Park(); break;
        }
    }

    public double getPotPosition() {
        double currVolts = potentiometer.getVoltage();
        double position = ((270*currVolts+445.5)-Math.sqrt(Math.pow(270*currVolts+445.5, 2) + 4*currVolts*(36450*currVolts-120285)))/(2*currVolts);
        return position - 15.06;
    }

    public void getPose() {
        limelight.getBotPose();
    }

    private void AimAndSpinUpConfig() {
        double command = controller.calculatePosition(shooterAngle, getPotPosition());
        pivotLeft.setPower(command);
        pivotRight.setPower(command);

        if (Math.abs(getPotPosition()) >= 95) {
            shootTop.setVelocity(shooterTopConfig);
            shootBottom.setVelocity(shooterBottomConfig);
//            Shoot();
        }
//        telemetry.addData("LL TY: ", limelight.getDistance(alliance_blue));
        telemetry.update();
    }

    /**
     * This action aims the shooter and spins up the motors
     * Uses odo position
     * Can be changed to use limelight distance to tag
     */
//    private void AimAndSpinUp() {
//        if (limelight.getDistance(alliance_blue) > 0) {
//            Result r = findBestShot(limelight.getDistance(alliance_blue), 15,
//                    0, 110, 43, 500, 1200,
//                    5, 10, 1.2);
//
//            shooterTopVelocity = r.topMotorDegPerSec;
//            shooterBottomVelocity = r.bottomMotorDegPerSec;
//
//            double command = controller.calculate(r.angleDeg, getPotPosition());
//            pivotLeft.setPower(command);
//            pivotRight.setPower(command);
//
//            if (Math.abs(r.angleDeg - getPotPosition()) <= 5) {
//                shootTop.setVelocity(shooterTopVelocity, AngleUnit.DEGREES);
//                shootBottom.setVelocity(shooterBottomVelocity, AngleUnit.DEGREES);
//            }
//        } else {
//            telemetry.addLine("To Close To Read Tag!");
//            telemetry.update();
//        }
//    }

    private void AimInPlaceFar() {
        shooterBottomVelocity = 1400;
        shooterTopVelocity = 1550;

        double command = controller.calculatePosition(110, getPotPosition());
        pivotLeft.setPower(command);
        pivotRight.setPower(command);

        if (Math.abs(getPotPosition()) >= 100) {
            telemetry.addLine("Spinning up wheels");
            shootTop.setVelocity(shooterTopVelocity);
            shootBottom.setVelocity(shooterBottomVelocity);
        }
    }
    private void AimInPlaceClose() {

        Double Ty = limelight.getTy();

        if (Ty == null)
        {
            shooterBottomVelocity = 1000;
        } else if (Ty < - 10) {
            shooterBottomVelocity = 1425;
        } else {
            shooterBottomVelocity = 3.5579 * Ty * Ty - 17.578 * Ty + 709.34;
        }
        shooterTopVelocity = 1550;

        double command = controller.calculatePosition(115, getPotPosition());
        pivotLeft.setPower(command);
        pivotRight.setPower(command);

        if (Math.abs(getPotPosition()) >= 105) {
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
    private int velocityTolCounter = 0;

    private boolean motorsAtVelocity(double targetTop, double targetBottom) {

        double top = shootTop.getVelocity();
        double bottom = shootBottom.getVelocity();
        if (Math.abs(top - targetTop) > VELOCITY_TOLERANCE ||
                Math.abs(bottom - targetBottom) > VELOCITY_TOLERANCE)
        {
            velocityTolCounter= 0;
        }
        else
        {
            velocityTolCounter ++;
        }
        boolean velocityState = velocityTolCounter > velocityTolTimeOut;
        if (velocityState)
        {
            velocityTolCounter = 0;
        }

        return (velocityState);
    }

    private void Shoot() {
        telemetry.addLine("Started shooting");
        if (shotOrder == null) {
            getServoOrder();
        }
//        Servo[] sequence = getServoOrder(shotOrder);
        Servo[] sequence = new Servo[]{kickLeft, kickCenter, kickRight};
        Double Ty = limelight.getTy();

        if (Ty == null)
        {
            shooterBottomVelocity = 1000;
        }
        else {
            shooterBottomVelocity = 3.5579 * Ty * Ty - 17.578 * Ty + 709.34;
        }
        shooterTopVelocity = 1550;

        // ResetAndWait until motors are at target velocity
        if (motorsAtVelocity(shooterTopVelocity, shooterBottomVelocity)) { //shooterTopVelocity, shooterBottomVelocity
            Servo servo = sequence[servoCounter];
            // Move the current servo
            servo.setPosition(0.85);
            telemetry.addLine("Moved servo");
            if (!waitStarted) {
                shootWaiter.startWait(250);
                waitStarted = true;
            }
            if (shootWaiter.isDone()) {
                servo.setPosition(0.65);
                servoCounter++;
                waitStarted = false;
            }
        } else {
            telemetry.addLine("Wheels not at speed!");
        }
        if (servoCounter > 2) {
            servoCounter = 0;
            currentAction = ShooterActions.ZeroPower;
        }
    }

    /**
     * This action shoots the balls in a specified order
     * Need to add a scan to find order tag
     */
    private void getServoOrder() {
        if (shotOrder == null) {
            greenShot first = limelight.getOrder();
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
//    private void alignAndAim() {
//        this.Drive.localizer.update();
//        //Target X - actual X, target Y - actual Y
//        double heading = Math.atan2((this.alliance_blue? blueGoalPose.x : redGoalPose.x) - limelight.getBotPose().position.x,
//                (this.alliance_blue? blueGoalPose.y : redGoalPose.y) - limelight.getBotPose().position.y);
//        this.Drive.new TurnAction(new TimeTurn(this.Drive.localizer.getPose(), heading, this.Drive.defaultTurnConstraints));
//        //AimAndSpinUp();
//    }

    private void Intake() {
        double command = controller.calculatePosition(inatkePos, getPotPosition());
        pivotLeft.setPower(command);
        pivotRight.setPower(command);
        kickCenter.setPosition(0.65);
        kickLeft.setPosition(0.65);
        kickRight.setPosition(0.65);
        shootTop.setPower(-0.5);
        shootBottom.setPower(0);
    }
    private void IntakeHuman() {
        double command = controller.calculatePosition(110, getPotPosition());
        pivotLeft.setPower(command);
        pivotRight.setPower(command);
        kickCenter.setPosition(0.65);
        kickLeft.setPosition(0.65);
        kickRight.setPosition(0.65);
        shootTop.setPower(-0.25);
        shootBottom.setPower(-0.25);
    }

    private void ZeroPower() {
        shootTop.setVelocity(0);
        shootBottom.setVelocity(0);
        double command = controller.calculatePosition(25, getPotPosition());
        kickCenter.setPosition(0.65);
        kickLeft.setPosition(0.65);
        kickRight.setPosition(0.65);
        pivotLeft.setPower(command);
        pivotRight.setPower(command);
    }

    private void Park() {
        shootTop.setVelocity(0);
        shootBottom.setVelocity(0);
        double command = controller.calculatePosition(110, getPotPosition());
        pivotLeft.setPower(command);
        pivotRight.setPower(command);
//        waiter.startWait(500);
//        if (waiter.isDone()) {
//          park.setPosition(0.1);
//        }
    }
}