package org.firstinspires.ftc.teamcode.MF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Cam Test")
public class CamTest extends OpMode {

    private ColorCam colorCam;

    public CamTest() {
    }

    @Override
    public void init() {
        colorCam = new ColorCam();
        colorCam.cameraInit(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("color", colorCam.getColor());
        telemetry.addData("hue", colorCam.getHue());
        telemetry.update();
    }
}

