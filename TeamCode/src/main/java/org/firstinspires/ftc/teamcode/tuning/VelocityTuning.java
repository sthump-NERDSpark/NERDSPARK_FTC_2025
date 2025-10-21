package org.firstinspires.ftc.teamcode.tuning;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.BasicSystem;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class VelocityTuning extends LinearOpMode{
    private DcMotorEx shooter = null;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0; // Velocity feedforward
    public static double kA = 0; // Acceleration feedforward
    public static double kS = 0; // static friction compensation
    public static double targetSpeed = 0;
    public static double actualSpeed = 0;
    public static double Q = 0.3; // High values put more emphasis on the sensor.
    public static double R = 3; // High Values put more emphasis on regression.
    public static int N = 3; // The number of estimates in the past we perform regression on.

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        shooter = hardwareMap.get(DcMotorEx.class, "shootTop");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(1);

        PIDCoefficients coefficients = new PIDCoefficients(kP,kI,kD);
        BasicPID controller = new BasicPID(coefficients);
        FeedforwardCoefficients feedforwardCof = new FeedforwardCoefficients(kV,kA,kS);
        BasicFeedforward feedforward = new BasicFeedforward(feedforwardCof);
        KalmanEstimator filter = new KalmanEstimator(() -> shooter.getVelocity(),Q,R,N);
        BasicSystem system = new BasicSystem(filter,controller,feedforward);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            double command = system.update(targetSpeed);
            shooter.setVelocity(command);
            actualSpeed = shooter.getVelocity();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Speed", targetSpeed);
            packet.put("Actual Speed", actualSpeed);
            packet.put("Estimated Speed", filter.measurement);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}