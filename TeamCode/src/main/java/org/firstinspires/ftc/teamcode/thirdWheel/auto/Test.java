package org.firstinspires.ftc.teamcode.thirdWheel.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;

@Autonomous(name="ThirdWheel Test", group="ThirdWheel")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveThirdWheel drive = new DriveThirdWheel(this, hardwareMap);
        double totalTime = 4.0;
        waitForStart();
        if (isStopRequested()) return;
        {
            ElapsedTime runtime = new ElapsedTime();
            while (runtime.seconds() < totalTime) {
                // telemetry.addData("celerate", drive.celerate(runtime.milliseconds(), 0.5, totalTime, 1000, 1000));
                drive.justX(runtime.seconds(), totalTime, 1);
            }
        }
        {
            ElapsedTime runtime = new ElapsedTime();
            while (runtime.seconds() < 6.0) {
                drive.fakeRoadrunner(runtime.seconds(), 2.0, -1, 4.0, 1, 6.0, 1);
            }
        }
    }
}
