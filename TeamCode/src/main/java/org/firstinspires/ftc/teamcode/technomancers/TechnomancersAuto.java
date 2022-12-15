package org.firstinspires.ftc.teamcode.technomancers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.technomancers.TechnomancersDrive;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Technomancers Auto", group="Technomancers")
public class TechnomancersAuto extends LinearOpMode {

    @Override public void runOpMode() throws InterruptedException {

        TechnomancersDrive drive = new TechnomancersDrive(this, hardwareMap);

        waitForStart();
        ElapsedTime e = new ElapsedTime();
        e.reset();
        e.startTime();
        while(e.time(TimeUnit.SECONDS) < 1.5) {
            drive.drive(0,0.3,0);
        }
        e.reset();
        //drive.move(0.4, 60, 0);
    }
}
