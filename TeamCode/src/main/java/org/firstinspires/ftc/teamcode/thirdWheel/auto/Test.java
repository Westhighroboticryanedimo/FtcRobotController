package org.firstinspires.ftc.teamcode.thirdWheel.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

import org.firstinspires.ftc.teamcode.thirdWheel.auto.actions.Barcode;

@Autonomous(name="ThirdWheel Test", group="ThirdWheel")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveThirdWheel drive = new DriveThirdWheel(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap);
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        Barcode barcode = new Barcode(this, drive, lift, distanceSensor);

        waitForStart();
        if (isStopRequested()) return;
        //drive.frrNormie(0, 0, 0, 8, 0, 1, 0.2, 0, 1);
        barcode.put();
//        drive.frrNormie(0, 0, 0, 16, 0, 1, 0.2, 0.25, 1);
        //drive.stop();
//        sleep(5000);
//        drive.frrNormie(0, 0, 0, 16, 0, -1, 0.2,  0, -1);
    }
}
