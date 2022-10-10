package org.firstinspires.ftc.teamcode.BoogerBoy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.slowBolon.DriveBolon;

@Autonomous(name = "minicat")
public class BoogerBoyAuto extends LinearOpMode{
    private BoogerBoyDrive drive;
    private int AI_resualt;
    // This is for when the cone is in the first case
    private void uno()
    {
        drive.move(1,60,0);
    }
    // This is for when the cone is in the seconds case
    private void dos()
    {
        drive.move(1,60,0);
        drive.move(1,60,90);
    }
    // This is for when the cone is in the third case
    private void tres()
    {
        drive.move(1,60,0);
        drive.move(1,60,-90);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        drive = new BoogerBoyDrive(this,hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        // AI Goes Here and it makes a decision between the options for thr cone
        // Some vision recognition detects one of three options and resualts 1,2,3
        AI_resualt = 1;

        // Theese detect what the AI Detected and run the corasponding code
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
