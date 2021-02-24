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

        drive.move(1, 20, 0);
        drive.turn(1, 9);

        if (numRingStack == 0) {

            // 0 ring

        } else if (numRingStack == 1) {

            // 1 rings

        } else {

            // 4 rings

        }

    }
}
