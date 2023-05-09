package org.firstinspires.ftc.teamcode.Wingman;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Wingman.subsystems.ColorCam;

@Disabled
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

