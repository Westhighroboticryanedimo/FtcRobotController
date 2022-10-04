package org.firstinspires.ftc.teamcode.fifthWheel.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fifthWheel.hardware.webcam.IntakeCam;

@Autonomous(name="FifthWheel Weegus", group="FifthWheel")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IntakeCam inCam = new IntakeCam(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
             telemetry.addData("Signal face", inCam.getSignalFace());
             telemetry.addData("Avg hue", inCam.getAvgHue());
//            telemetry.addData("sussus", "awogu");
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(200);
        }
    }
}
