package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp
public class VelocityTuning extends LinearOpMode{
    DcMotorEx shooter = null;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;
    public static double targetSpeed = 0;
    public static double actualSpeed = 0;
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        shooter = hardwareMap.get(DcMotorEx.class, "shootTop");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPower(1);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            shooter.setVelocityPIDFCoefficients(kP,kI,kD,kF);
            shooter.setVelocity(targetSpeed);
            actualSpeed = shooter.getVelocity();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Speed", targetSpeed);
            packet.put("Actual Speed", actualSpeed);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
