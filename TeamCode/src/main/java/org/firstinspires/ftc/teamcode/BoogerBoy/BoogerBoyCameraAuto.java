package org.firstinspires.ftc.teamcode.BoogerBoy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.BoogerBoy.hardware.BoogerCam;

@Autonomous(name="Nuclear Detonation Autonomous")
public class BoogerBoyCameraAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BoogerCam cam = new BoogerCam(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Signal face", cam.getFaceFace());
            telemetry.addData("color being seen", cam.getAvgHue());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(200);
        }
    }
}
