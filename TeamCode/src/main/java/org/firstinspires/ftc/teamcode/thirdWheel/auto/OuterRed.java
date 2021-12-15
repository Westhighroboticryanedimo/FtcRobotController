package org.firstinspires.ftc.teamcode.thirdWheel.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Gyro;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

@Autonomous(name="ThirdWheel OuterRed", group="ThirdWheel")
public class OuterRed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveThirdWheel drive = new DriveThirdWheel(this, hardwareMap);
        Lift lift = new Lift(this, hardwareMap);

        // Elapsed time for timed motion
        ElapsedTime runtime = new ElapsedTime();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (runtime.seconds() < 6) {
            lift.override(1, -1);
            lift.assist();
            drive.setPowers(0.5, -0.5, -0.5, 0.5);
        }
        drive.stop();
        lift.override(0, -1);
        lift.assist();
    }

}


