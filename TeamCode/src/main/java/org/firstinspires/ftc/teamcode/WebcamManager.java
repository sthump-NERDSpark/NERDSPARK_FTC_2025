package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class WebcamManager {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    public void init(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public Shooter.greenShot getOrder() {
        detectedTags = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detectedTags) {
            int id = detection.id; // The ID number of the fiducial
            if (id == 22) {
                return Shooter.greenShot.SECOND;
            } else if (id == 23) {
                return Shooter.greenShot.THIRD;
            } else {
                return Shooter.greenShot.FIRST;
            }
        }
        return null;
    }

    public Vector2d getDistance(boolean alliance_blue) {
        detectedTags = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detectedTags) {
            int id = detection.id;
            if (id == 20 && alliance_blue) {
                AprilTagPoseFtc pose = detection.ftcPose;
                return new Vector2d(pose.x, pose.y);
            } else if (id == 24 && !alliance_blue) {
                AprilTagPoseFtc pose = detection.ftcPose;
                return new Vector2d(pose.x, pose.y);
            }
        }
        return null;
    }
}
