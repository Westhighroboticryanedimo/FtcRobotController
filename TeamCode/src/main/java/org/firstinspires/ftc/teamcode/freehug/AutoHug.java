package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="autohug")
public class AutoHug extends LinearOpMode {
    GrabberFree grabber;

    public void lowerArmAuto(int ms) {
        grabber.lowerHand();
        sleep(ms);
        grabber.restElbow();
    }
    public void raiseArmAuto(int ms) {
        grabber.handInTheAir();
        sleep(ms);
        grabber.restElbow();
    }

    public void openHandAuto() { grabber.openHand(); }
    public void closeHandAuto() { grabber.closeHand(); }

    @Override
    public void runOpMode() throws InterruptedException {
        Freehugdrive drive = new Freehugdrive(this, hardwareMap);
        WebcamFree webcam = new WebcamFree(this, hardwareMap);
        grabber = new GrabberFree(this, hardwareMap);
        IntakeFree intake = new IntakeFree(this, hardwareMap);
        
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        // Get how many rings are stacked
        int rings = 0;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {

            rings = webcam.getNumRings();

            telemetry.addData("Num rings", webcam.getNumRings());
            telemetry.addData("Ring position", webcam.getInternalRingNum());
            telemetry.update();

        }

        //easy ato
    //drive.move(0.6,72,180);
        //random stuff
        //sleep(500);
        //drive.turn(1,360);
        //sleep(500);
        //lowerArmAuto(1000);
        //sleep(1000);
        //raiseArmAuto(1000);

        //WEIRD NUMBERS
        if (rings == 0) {

            // 0 ring
            drive.move(0.6,83,180);
            sleep(1000);
            drive.move(0.6, 25, -90);
        } else if (rings == 1) {

            // 1 rings
            drive.move(.6, 98, 180);
            sleep(2000);
            drive.move(.6, 16, 0);
        } else {

            // 4 rings
            drive.move(0.6,130,180);
            sleep(1000);
            drive.move(0.6, 25, -90);
            drive.move(0.6,45,0);
        }


        /*
        //ACTUAL MEASUREMENTS
        if(rings == 0) {
            drive.move(0.6,74,180);
            sleep(1000);
            drive.move(0.6, 25, -90);
        }
        else if(rings == 1) {
            drive.move(.6, 92, 180);
            sleep(1000);
            drive.move(.6, 16, -90);
        } else {
            drive.move(0.6,112,180);
            sleep(1000);
            drive.move(0.6, 25, -90);
        }
        */
    }
}
