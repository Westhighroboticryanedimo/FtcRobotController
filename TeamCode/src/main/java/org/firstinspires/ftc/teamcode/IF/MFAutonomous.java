package org.firstinspires.ftc.teamcode.IF;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MFTonomous")

public class MFAutonomous extends LinearOpMode {

    MFDrive drive;

    @Override public void runOpMode() throws InterruptedException {
        waitForStart();
        drive.move(0.4, 60, 0);
    }
}