package org.firstinspires.ftc.teamcode.thirdWheel.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Gyro;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

@Autonomous(name="ThirdWheel InnerBlue", group="ThirdWheel")
public class InnerBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveThirdWheel drive = new DriveThirdWheel(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap);

        drive.debug();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        lift.override(1, -1);
        lift.assist(); // to set level and stuff
        while (!lift.arrived()) {
            lift.assist();
        }
        drive.move(0.5, 40, 270);
        // drive.move(0.5, 12, 0);
        // this doesn't work
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < 0.75) {
            drive.setPowers(0.5, 0.5, 0.5, 0.5);
        }
        drive.stop();
        lift.override(0, -1);
        lift.assist();
        while (runtime.seconds() < 5) {
            lift.assist();
        }
    }
}


