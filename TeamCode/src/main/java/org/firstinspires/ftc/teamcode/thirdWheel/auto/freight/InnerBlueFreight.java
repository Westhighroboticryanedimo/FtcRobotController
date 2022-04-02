package org.firstinspires.ftc.teamcode.thirdWheel.auto.freight;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

import org.firstinspires.ftc.teamcode.thirdWheel.auto.actions.Barcode;
import org.firstinspires.ftc.teamcode.thirdWheel.auto.actions.Park;
import org.firstinspires.ftc.teamcode.thirdWheel.auto.actions.Freight;

@Autonomous(name="ThirdWheel InnerBlueFreight", group="ThirdWheel")
public class InnerBlueFreight extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveThirdWheel drive = new DriveThirdWheel(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap);
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        Barcode barcode = new Barcode(this, drive, lift, distanceSensor);
        Park park = new Park(this, drive, lift);
        Freight freight = new Freight(this, drive, lift);

        waitForStart();
        if (isStopRequested()) return;

//        barcode.setup(1);
//        barcode.detect();
//        barcode.put();
//        park.warehouse(1);
//        lift.override(1, -1);
//        lift.assist();
//        while (!lift.arrived()) {
//            lift.assist();
//        }
        freight.setup(1);
        freight.move(1);
        freight.pick();
        freight.move(-1);
    }

}
