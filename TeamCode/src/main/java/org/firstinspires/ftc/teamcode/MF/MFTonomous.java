package org.firstinspires.ftc.teamcode.MF;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Read/Park")

public class MFTonomous extends LinearOpMode {

    ColorCam colorCam = new ColorCam();

    @Override public void runOpMode() throws InterruptedException {
        colorCam.cameraInit(hardwareMap);
        telemetry.addData("color", colorCam.getColor());
        telemetry.addData("hue", colorCam.getHue());
        telemetry.update();

        waitForStart();

    }
}