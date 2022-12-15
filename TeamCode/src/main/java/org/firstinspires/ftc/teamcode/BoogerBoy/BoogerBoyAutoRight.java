package org.firstinspires.ftc.teamcode.BoogerBoy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BoogerBoy.hardware.BoogerCam;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "田ロ田 Booger Boy Auto [RIGHT PRELOAD] ロ田ロ")
public class BoogerBoyAutoRight extends LinearOpMode{
    private BoogerBoyDrive drive;
    private Servo grabby;
    private DcMotor lift;

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
        grabby = hardwareMap.get(Servo.class,"grabby");
        lift = hardwareMap.get(DcMotor.class,"lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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
        // These detect what the AI Detected and run the corresponding code
        drive.move(0.7,52,180);
        drive.move(0.7,52,0); // right
/*
        grabby.setPosition(0.5);
        sleep(555);
        drive.move(0.7,13,90);
        while(lift.getCurrentPosition()<3000) {lift.setPower(1);sleep(10);}
        lift.setPower(0);
        drive.move(0.7,31,270);
        drive.turn(0.6,-90);
        while(lift.getCurrentPosition()<3000) {lift.setPower(1);sleep(10);}
        lift.setPower(0);
        drive.move(0.7,6,270);
        grabby.setPosition(1);
        drive.move(0.7,10,90);
        drive.turn(0.6,90);
        drive.move(0.7,63,270);
*/
        // back to starting position

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
