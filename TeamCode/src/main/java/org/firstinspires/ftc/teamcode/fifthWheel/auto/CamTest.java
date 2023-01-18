package org.firstinspires.ftc.teamcode.fifthWheel.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.IntakeCam;

import java.lang.Math;

@Autonomous(name="CamTest", group="FifthWheel")
public class CamTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        IntakeCam inCam = new IntakeCam(hardwareMap, false);
        Controller controller = new Controller(gamepad1);
        Gyro gyro = new Gyro(hardwareMap);

        int mode = 0;

        FtcDashboard.getInstance().startCameraStream(IntakeCam.camera, 0);

        while (!isStarted() && !isStopRequested()) {
            controller.update();
            if (controller.A()) {
                mode = 0;
            } else if (controller.B()) {
                mode = 1;
            }
            if (controller.Y()) {
                inCam.beginConeStack();
            } else if (controller.X()) {
                inCam.beginSignalDetection();
            }
            if (mode == 0) {
                if (controller.dpadUp()) {
                    inCam.changeCornerY(5);
                }
                if (controller.dpadDown()) {
                    inCam.changeCornerY(-5);
                }
                if (controller.dpadLeft()) {
                    inCam.changeCornerX(-5);
                }
                if (controller.dpadRight()) {
                    inCam.changeCornerX(5);
                }
            }
            if (mode == 1) {
                if (controller.dpadUp()) {
                    inCam.changeRegionHeight(-5);
                }
                if (controller.dpadDown()) {
                    inCam.changeRegionHeight(5);
                }
                if (controller.dpadLeft()) {
                    inCam.changeRegionWidth(-5);
                }
                if (controller.dpadRight()) {
                    inCam.changeRegionWidth(5);
                }
            }
            telemetry.addData("regionWidth", inCam.getRegionWidth());
            telemetry.addData("regionHeight", inCam.getRegionHeight());
            telemetry.addData("cornerX", inCam.getCornerX());
            telemetry.addData("cornerY", inCam.getCornerY());
            telemetry.addData("Signal face", inCam.getSignalFace());
            telemetry.addData("Avg hue", inCam.getAvgHue());
            telemetry.addData("pixelWidth", inCam.getPixelWidth());
            telemetry.addData("midWidth", inCam.getMidWidth());
            telemetry.addData("y distance", inCam.getYDistance());
            telemetry.addData("x distance", inCam.getXDistance(gyro.getAngleRadians()));
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(200);
        }
    }
}
