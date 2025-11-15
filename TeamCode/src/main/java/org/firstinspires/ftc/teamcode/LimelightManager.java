package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.PID;

import java.util.List;

@Config
public class LimelightManager {
    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private final boolean alliance_blue;
    private PID pid;
    public static double kp = 0.02;
    public static double ki = 0.02;
    public static double kd = 0.00000008;

    public LimelightManager(HardwareMap hardwareMap, Telemetry Telemetry, boolean alliance) {
        this.telemetry = Telemetry;
        this.alliance_blue = alliance;

//        pid = new PID(0.02, 0, 0.0000001);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(250); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0);
        limelight.start(); // This tells Limelight to start looking!
    }

    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    public void getBotPose() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            telemetry.addData("TX, TY: ", "(" + result.getTx() + "," + result.getTy() + ")");
            double Ty = result.getTy();
        }
    }

     public double angleToGoalRED() {
        LLResult result = limelight.getLatestResult();
        pid = new PID(kp, ki, kd);
        if (result.isValid()) {
            return pid.calculate(result.getTx(), 0);
        }
        return -10;
    }

    public double angleToGoalBLUE() {
        LLResult result = limelight.getLatestResult();
        pid = new PID(kp, ki, kd);
        if (result.isValid()) {
            return pid.calculate(result.getTx(), 0);
        }
        return -10;
    }

    public Double getTy() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            return result.getTy();
        }

        // return null (or Double.NaN) when there's no valid target
        return null;
    }
}