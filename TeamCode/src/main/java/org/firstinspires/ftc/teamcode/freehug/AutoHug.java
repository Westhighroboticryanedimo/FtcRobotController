package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.marinara.hardware.Webcam;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
@Autonomous(name="autohug")
public class AutoHug extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Freehugdrive drive = new Freehugdrive(this, hardwareMap);
        GrabberFree grabber = new GrabberFree(this, hardwareMap);
        IntakeFree intake = new IntakeFree(this, hardwareMap);
        Webcam webcam = new Webcam(this, hardwareMap);

        /*drive.move(1, 20, 0);
        drive.turn(1, 9);*/

        int rings;
        rings = webcam.getNumRings();

        //drive.move(1,72,180);



        /*
        if(rings ==) {
            drive.move(0.6,130,180);
            sleep(1000);
            drive.move(0.6, 30, -90);
        }
        else if(rings ==) {
            drive.move(.6, 98, 180);
            sleep(1000);
            drive.move(.6, 16, 0);
        } else if(rings ==) {
            drive.move(0.6,83,180);
            sleep(1000);
            drive.move(0.6, 25, -90);
        }
        */



        /*
        //actual measurements
        if(rings == ) {
            drive.move(0.6,74,180);
            sleep(1000);
            drive.move(0.6, 25, -90);
        }
        else if(rings ==) {
            drive.move(.6, 92, 180);
            sleep(1000);
            drive.move(.6, 12, 0);
        } else if(rings == ) {
            drive.move(0.6,112,180);
            sleep(1000);
            drive.move(0.6, 25, -90);
        }
        */
    }
}
