package org.firstinspires.ftc.teamcode.marinara.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.marinara.hardware.UltimateGoalDrive;

@Autonomous(name = "Ultimate Goal All Auto", group = "UltimateGoal")
//@Disabled
public class AllAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        UltimateGoalDrive drive = new UltimateGoalDrive(this, hardwareMap);
        drive.debug();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        drive.move(1.0, 50, 120);
        drive.move(1.0, 20, 60);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
