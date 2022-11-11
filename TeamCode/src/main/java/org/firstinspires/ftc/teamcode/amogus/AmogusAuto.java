package org.firstinspires.ftc.teamcode.amogus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name="Amogus")
public class AmogusAuto extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();
        if (isStopRequested()) return;

        servo.setPosition(0);
        servo.setPosition(1);
        servo.setPosition(0);
        // later
        servo.setPosition(0);
        servo.setPosition(0.25);
        servo.setPosition(0.5);
        servo.setPosition(1);
    }
}
