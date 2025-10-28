package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class PositionTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        AnalogInput potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            double currVolts = potentiometer.getVoltage();
            double position = ((270*currVolts+445.5)-Math.sqrt(Math.pow(270*currVolts+445.5, 2) + 4*currVolts*(36450*currVolts-120285)))/(2*currVolts);
            telemetry.addData("Shooter Position (Deg): ", position);
            telemetry.addData("Current Volts: ", currVolts);
            telemetry.update();
        }
    }
}