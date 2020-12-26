package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.drive.AcmeHolonomicDrive;

@Config
@Autonomous(group = "Test")
@Disabled
public class TurnTest extends LinearOpMode {

    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {

        AcmeHolonomicDrive drive = new AcmeHolonomicDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));

    }

}
