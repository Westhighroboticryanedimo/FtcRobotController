package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
@Autonomous(name="autohug")
public class AutoHug extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Freehugdrive drive = new Freehugdrive(this, hardwareMap);
        WebcamFree webcam = new WebcamFree(this, hardwareMap);
        GrabberFree grabber = new GrabberFree(this, hardwareMap);
        IntakeFree intake = new IntakeFree(this, hardwareMap);

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        // Get how many rings are stacked
        int numRingStack = 0;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {

            numRingStack = webcam.getNumRings();

            telemetry.addData("Num rings", webcam.getNumRings());
            telemetry.addData("Ring position", webcam.getInternalRingNum());
            telemetry.update();

        }

        /*drive.move(1, 20, 0);
        drive.turn(1, 9);*/

        drive.move(0.6,130,180);
        sleep(1000);
        drive.move(0.6, 30, -90);

        if (numRingStack == 0) {

            // 0 ring

        } else if (numRingStack == 1) {

            // 1 rings

        } else {

            // 4 rings

        }

        /*if() {

        }
        else if() {
            drive.move(.5, 98, 180);
            sleep(2000);
            drive.move(.5, 16, 0);
        } else if() {
            drive.move(0.6,83,180);
            sleep(1000);
            drive.move(0.6, 25, -90);

        }*/

    }
}
