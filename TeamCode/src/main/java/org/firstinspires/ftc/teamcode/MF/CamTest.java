package org.firstinspires.ftc.teamcode.MF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MF.subsystems.ColorCam;

@TeleOp(name = "Cam Test")
public class CamTest extends OpMode {

    ColorCam colorCam = new ColorCam();

    public CamTest() {
    }

    @Override
    public void init() {
        colorCam.cameraInit(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("color", colorCam.getColor());
        telemetry.addData("hue", colorCam.getHue());
        telemetry.update();
    }
}

