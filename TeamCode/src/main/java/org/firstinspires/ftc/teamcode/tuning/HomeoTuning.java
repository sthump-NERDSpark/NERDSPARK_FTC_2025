package org.firstinspires.ftc.teamcode.tuning;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.NoFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.RawValue;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.function.DoubleSupplier;

@Config
@TeleOp
public class HomeoTuning extends LinearOpMode{
    private DcMotorEx pivotLeft = null;
    private DcMotorEx pivotRight = null;
    private AnalogInput potentiometer = null;
    public static double currVolts = 0;
    public static double position = 0;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double targetPosition = 50;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        pivotLeft = hardwareMap.get(DcMotorEx.class, "leftPivot");
        pivotRight = hardwareMap.get(DcMotorEx.class, "rightPivot");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        pivotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotRight.setDirection(DcMotorSimple.Direction.FORWARD);
        pivotLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            currVolts = potentiometer.getVoltage();
            position = ((270*currVolts+445.5)-Math.sqrt(Math.pow(270*currVolts+445.5, 2) + 4*currVolts*(36450*currVolts-120285)))/(2*currVolts);

            PIDCoefficients coefficients = new PIDCoefficients(kP,kI,kD);
            BasicPID controller = new BasicPID(coefficients);
            DoubleSupplier systemPosition = () -> (position + 0.8653);
            NoFeedforward feedforward = new NoFeedforward();
            RawValue noFilter = new RawValue(systemPosition);
            BasicSystem system = new BasicSystem(noFilter,controller,feedforward);

            double command = system.update(targetPosition);
            pivotLeft.setPower(command);
            pivotRight.setPower(command);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Position", targetPosition);
            packet.put("Actual Position", position + 0.8653);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}