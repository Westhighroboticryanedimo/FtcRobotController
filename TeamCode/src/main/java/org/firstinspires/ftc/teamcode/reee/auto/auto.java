package org.firstinspires.ftc.teamcode.reee.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.hardware.Gyro;

import java.util.ArrayList;

@Autonomous(name="reee auto", group="reee")
public class auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // DistanceSensor ds = hardwareMap.get(DistanceSensor.class, "ds");
        ElapsedTime runtime = new ElapsedTime();
        Servo rotator = hardwareMap.get(Servo.class, "rotator");
//        ArrayList<Double> values = new ArrayList<Double>();
        double pos = 0.023;
        waitForStart();
        if (isStopRequested()) return;

//        rotator.setPosition(0.023);
//        telemetry.addData("0", 0);
//        sleep(2000);
//        rotator.setPosition(0.341);
//        telemetry.addData("90", 90);
//        sleep(2000);
//        rotator.setPosition(0.023);
//        telemetry.addData("0", 0);
//        sleep(2000);
//        rotator.setPosition(0.341);
//        telemetry.addData("90", 90);
//        sleep(2000);
//        rotator.setPosition(0.023);
//        telemetry.addData("0", 0);
//        sleep(2000);

        while (pos <= 0.341) {
            pos += 0.02;
            rotator.setPosition(pos);
            sleep(100);
        }
        sleep(500);
        while (pos > 0.003) {
            pos -= 0.02;
            rotator.setPosition(pos);
            sleep(100);
        }
        sleep(500);
//        sleep(1000);
//
//        rotator.setPosition(0.023);
//        telemetry.addData("0", 0);
//        sleep(2000);
//        rotator.setPosition(1);
//        telemetry.addData("0", 0);
//        sleep(2000);

//        while (time.seconds() < 10) {
//            values.add(ds.getDistance(DistanceUnit.INCH));
//        }
//        telemetry.addData("count:", values.size());
//        telemetry.update();
//        sleep(2000);
    }
}

