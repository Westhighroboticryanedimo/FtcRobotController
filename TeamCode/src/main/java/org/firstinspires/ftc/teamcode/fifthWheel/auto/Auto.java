package org.firstinspires.ftc.teamcode.fifthWheel.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.fifthWheel.hardware.DriveFifthWheel;
import org.firstinspires.ftc.teamcode.fifthWheel.hardware.webcam.IntakeCam;

@Autonomous(name="FifthWheel Auto", group="FifthWheel")
public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveFifthWheel drive = new DriveFifthWheel(this, hardwareMap);
        IntakeCam inCam = new IntakeCam(hardwareMap);
        Controller controller = new Controller(gamepad1);
        int mode = 0;

        sleep(1000);
        int signal = inCam.getSignalFace();

        while (!isStarted() && !isStopRequested()) {
            controller.update();
            if (controller.A()) {
                mode = 0;
            } else if (controller.B()) {
                mode = 1;
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
            telemetry.addData("actual", signal);
            telemetry.addData("Avg hue", inCam.getAvgHue());
//            telemetry.addData("sussus", "awogu");
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(200);
        }

        signal = inCam.getSignalFace();

//        waitForStart();
//        if (isStopRequested()) return;
        switch (signal) {
            case 1:
                drive.move(0.5, 40, 90);
                drive.move(0.5, 40, 180);
                break;
            case 2:
                drive.move(0.5, 40, 180);
                break;
            case 3:
                drive.move(0.5, 40, -90);
                drive.move(0.5, 40, 180);
                break;
        }
    }
}
