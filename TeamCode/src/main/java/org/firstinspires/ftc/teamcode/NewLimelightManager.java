package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.PID;

import java.util.List;

@Config
public class NewLimelightManager {
    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private final boolean alliance_blue;
    private final MecanumDrive Drive;
    private PID pid;
    public static double kp = 0.02;
    public static double ki = 0.02;
    public static double kd = 0.00000008;

    public NewLimelightManager(HardwareMap hardwareMap, Telemetry Telemetry, boolean alliance, MecanumDrive drive) {
        this.telemetry = Telemetry;
        this.alliance_blue = alliance;
        this.Drive = drive;

//        pid = new PID(0.02, 0, 0.0000001);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(250); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(2);
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

    public Shooter.greenShot getOrder() {
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
        return Shooter.greenShot.FIRST;
    }

    public double angleToGoalBLUE() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            Drive.localizer.setPose(new Pose2d(result.getBotpose().getPosition().x, result.getBotpose().getPosition().y,
                    Drive.localizer.getPose().heading.toDouble()));
//            double heading = Math.atan((-1.83 - Drive.localizer.getPose().position.y) / (-1.83 - Drive.localizer.getPose().position.x));
//            telemetry.addData("Atan Heading: ", heading);
//            return heading;
        }
//        return -10;
        double heading = Math.atan((-1.83 - Drive.localizer.getPose().position.y) / (-1.83 - Drive.localizer.getPose().position.x));
        double headingDegrees = -(heading * (180/Math.PI)) - 45;
        telemetry.addData("Atan Heading: ", headingDegrees);
        return headingDegrees;
    }

    public double angleToGoalRED() {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            Drive.localizer.setPose(new Pose2d(result.getBotpose().getPosition().x, result.getBotpose().getPosition().y,
                    Drive.localizer.getPose().heading.toDouble()));
//            double heading = Math.atan((-1.83 - Drive.localizer.getPose().position.y) / (1.83 - Drive.localizer.getPose().position.x));
//            telemetry.addData("Atan Heading: ", heading);
//            return heading;
        }
//        return -10;
        double heading = Math.atan((-1.83 - Drive.localizer.getPose().position.y) / (1.83 - Drive.localizer.getPose().position.x));
        double headingDegrees = heading * (180/Math.PI);
        telemetry.addData("Atan Heading: ", headingDegrees);
        return headingDegrees;
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
