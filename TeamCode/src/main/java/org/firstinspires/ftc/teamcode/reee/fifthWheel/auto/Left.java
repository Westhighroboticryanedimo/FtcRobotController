package org.firstinspires.ftc.teamcode.reee.fifthWheel.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.reee.fifthWheel.subsystem.DriveFifthWheel;
import org.firstinspires.ftc.teamcode.reee.fifthWheel.subsystem.IntakeCam;
import org.firstinspires.ftc.teamcode.reee.fifthWheel.command.Place;

@Autonomous(name="FifthWheel Left", group="FifthWheel")
public class Left extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveFifthWheel drive = new DriveFifthWheel(this, hardwareMap);
        IntakeCam inCam = new IntakeCam(hardwareMap);
        Place place = new Place(hardwareMap, "leftMotor", "rightMotor", "flipLeft", "flipRight", "grip");
        Controller controller = new Controller(gamepad1);
        int mode = 0;
        int signal = 1;

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
            telemetry.addData("Avg hue", inCam.getAvgHue());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(200);
        }

        signal = inCam.getSignalFace();

        drive.move(0.1, 4, 0);
        drive.move(0.2, 6, 0);
        drive.move(0.5, 20, 0);
        drive.move(0.2, 6, 0);
        drive.move(0.1, 4, 0);
        drive.stop();
        sleep(100);

        drive.turn(0.1, -10);
        drive.turn(0.2, -20);
        drive.turn(0.5, -30);
        drive.turn(0.2, -20);
        drive.turn(0.1, -10);
        drive.stop();
        sleep(100);

        drive.move(0.1, 4, 180);
        drive.move(0.2, 6, 180);
        drive.move(0.5, 20, 180);
        drive.move(0.2, 6, 180);
        drive.move(0.1, 4, 180);
        drive.stop();
        sleep(100);

        drive.turn(0.1, -5);
        drive.turn(0.2, -10);
        drive.turn(0.5, -15);
        drive.turn(0.2, -10);
        drive.turn(0.1, -5);
        drive.stop();
        sleep(100);

        // switch (signal) {
        //     case 1:
        //         drive.move(0.5, 40, -90);
        //         drive.move(0.5, 40, 0);
        //         break;
        //     case 2:
        //         drive.move(0.5, 40, 0);
        //         break;
        //     case 3:
        //         drive.move(0.5, 40, 90);
        //         drive.move(0.5, 40, 0);
        //         break;
        // }
    }
}
