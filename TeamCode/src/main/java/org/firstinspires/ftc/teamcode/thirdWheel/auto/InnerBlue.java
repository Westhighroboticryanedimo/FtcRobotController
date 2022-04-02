package org.firstinspires.ftc.teamcode.thirdWheel.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Gyro;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

import org.firstinspires.ftc.teamcode.thirdWheel.auto.actions.Park;

@Autonomous(name="ThirdWheel InnerBlue", group="ThirdWheel")
public class InnerBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveThirdWheel drive = new DriveThirdWheel(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap);
        Park park = new Park(this, drive, lift);
//
//        telemetry.addData("foo", 0);
//        telemetry.update();

        waitForStart();
//        telemetry.addData("foo", 1);
//        telemetry.update();
        if (isStopRequested()) return;
//        telemetry.addData("foo", 2);
//        telemetry.update();
//
//        telemetry.addData("ticks", lift.getCurrentTicks());
//        telemetry.update();
        lift.override(1, -1);
        lift.assist();
        ElapsedTime time = new ElapsedTime();
        while (time.seconds() < 3) {
            lift.assist();
        }
//        drive.move(0.5, 4, 0);
    }
}
