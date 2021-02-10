package org.firstinspires.ftc.teamcode.marinara.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.marinara.hardware.MarinaraDrive;

@Autonomous(name = "Marinara All Auto", group = "Marinara")
//@Disabled
public class AllAuto extends LinearOpMode {

    private MarinaraDrive drive;

    @Override
    public void runOpMode() {

        drive = new MarinaraDrive(this, hardwareMap);
        drive.debug();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        drive.move(1.0, 50, -45, -90);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
