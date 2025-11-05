package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Limelight Teleop", group = "Robot")

public class LimelightTeleop extends LinearOpMode{
    private Limelight3A limelight;
    BNO055IMU imu;


    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start(); // This tells Limelight to start looking!

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /*
         * Starts polling for data.
         */
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
           LLResult result = limelight.getLatestResult();
            if (result != null) {
               if (result.isValid()) {
                    telemetry.addData("tid", result.getFiducialResults().get(0).getFiducialId());
                    telemetry.addData("botpose", result.getBotpose());
                }
            } else {
                    telemetry.addData("tid", "none");
            }

            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                }
            }

            double robotYaw = imu.getAngularOrientation().firstAngle;
            limelight.updateRobotOrientation(robotYaw);
            telemetry.addData("yaw", robotYaw);

            if (result != null && result.isValid()) {
                Pose3D botpose_mt2 = result.getBotpose_MT2();
                if (botpose_mt2 != null) {
                    double x = botpose_mt2.getPosition().x;
                    double y = botpose_mt2.getPosition().y;
                    telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                }
            }
        telemetry.update();
        }
    }
}
