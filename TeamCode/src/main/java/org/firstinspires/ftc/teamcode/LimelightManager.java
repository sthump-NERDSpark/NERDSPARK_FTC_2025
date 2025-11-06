package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightManager {
    Limelight3A limelight;

    public LimelightManager(@NonNull HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(250); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0);
        limelight.start(); // This tells Limelight to start looking!
    }
    public Pose2d getBotPose(MecanumDrive drive, @NonNull Telemetry telemetry) {
        // First, tell Limelight which way your robot is facing
        double robotYaw = Math.toDegrees(drive.localizer.getPose().heading.toDouble());
        limelight.updateRobotOrientation(robotYaw);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                return new Pose2d(x, y, robotYaw);
            }
        }
        return null;
    }

    public Shooter.greenShot getOrder(boolean alliance_blue) {
        double fieldRight = alliance_blue? 90 : 270;
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double tagYaw = fiducial.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES);
            if (id == 22 && Math.abs(fieldRight - tagYaw) <= 25) {
                return Shooter.greenShot.SECOND;
            } else if (id == 23 && Math.abs(fieldRight - tagYaw) <= 25) {
                return Shooter.greenShot.THIRD;
            } else {
                return Shooter.greenShot.FIRST;
            }
        }
        return null;
    }

//    takes distance from goal based on alliance
    public double getDistance(boolean alliance_blue) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id == 20 && alliance_blue) {
                    return (50.8/Math.tan((fiducial.getTargetArea()/100) * 27.2525)) *
                            (1/(Math.cos(fiducial.getTargetXDegrees()) * Math.cos(fiducial.getTargetYDegrees())));
                } else if (id == 24 && !alliance_blue) {
                    return (50.8/Math.tan((fiducial.getTargetArea()/100) * 27.2525)) *
                            (1/(Math.cos(fiducial.getTargetXDegrees()) * Math.cos(fiducial.getTargetYDegrees())));
                }
            }
        }
        return -1;
    }
}
