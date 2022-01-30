package org.firstinspires.ftc.teamcode.thirdWheel.auto.barcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

import org.firstinspires.ftc.teamcode.thirdWheel.auto.actions.Barcode;
import org.firstinspires.ftc.teamcode.thirdWheel.auto.actions.Park;

@Autonomous(name="ThirdWheel OuterBlueBarcode", group="ThirdWheel")
public class OuterBlueBarcode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveThirdWheel drive = new DriveThirdWheel(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap);
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        Barcode barcode = new Barcode(this, drive, lift, distanceSensor);
        Park park = new Park(this, drive, lift);

        waitForStart();
        if (isStopRequested()) return;
        barcode.setup();
        barcode.detect(2);
        barcode.put(2);
        park.warehouse(1);
    }

}


