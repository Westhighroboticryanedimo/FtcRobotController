package org.firstinspires.ftc.teamcode.BoogerBoy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BoogerBoy.hardware.BoogerCam;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "田ロ田 Booger Boy Auto [LEFT PRELOAD] 田ロ")
public class BoogerBoyAutoLeft extends LinearOpMode{
    private BoogerBoyDrive drive;

    // makeing a webcam for open cv
    OpenCvWebcam webcam;

    private int AI_resualt;
    // This is for when the cone is in the first case

    private void uno()
    {
        drive.move(0.7,52,270);
        drive.move(0.7,60,0);

    }
    // This is for when the cone is in the second case
    private void dos()
    {
        drive.move(1,52,270);
    }
    // This is for when the cone is in the third case
    private void tres()
    {
        drive.move(0.7,52,270);
        drive.move(0.7,60,180);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Start the pipeline for OpenCV



            // AI Goes Here ^ ^ ^ ^

        AI_resualt = 999; // resualt is not a word.
        BoogerCam cam = new BoogerCam(hardwareMap);
        drive = new BoogerBoyDrive(this,hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Signal face", cam.getFaceFace());
            AI_resualt = cam.getFaceFace();
            telemetry.addData("color being seen", cam.getAvgHue());
            telemetry.addData("face: ",AI_resualt);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(200);
        }
        telemetry.addData("face: ",AI_resualt);
        telemetry.update();
        // Theese detect what the AI Detected and run the corasponding code

        drive.move(0.7,52,0);
        drive.move(0.7,52,180); // left

        if(AI_resualt == 1)
        {
            uno();
        }
        if(AI_resualt == 2)
        {
            dos();
        }
        if(AI_resualt == 3)
        {
            tres();
        }



    }
}
