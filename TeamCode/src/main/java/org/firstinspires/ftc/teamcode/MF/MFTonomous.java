package org.firstinspires.ftc.teamcode.MF;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Read/Park")

public class MFTonomous extends LinearOpMode {

    MFDrive drive;

    @Override public void runOpMode() throws InterruptedException {
        waitForStart();
        drive.move(0.4, 60, 0);
    }
}