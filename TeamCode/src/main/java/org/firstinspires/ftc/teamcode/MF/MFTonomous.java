package org.firstinspires.ftc.teamcode.MF;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MF.webcam;

@Autonomous(name = "Read/Park")

public class MFTonomous extends LinearOpMode {

    MFDrive drive = new MFDrive(this, hardwareMap);

    @Override public void runOpMode() throws InterruptedException {
        waitForStart();

        drive.move(0.4, 60, 0);
    }
}