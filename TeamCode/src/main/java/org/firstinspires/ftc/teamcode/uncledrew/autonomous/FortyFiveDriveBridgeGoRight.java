package org.firstinspires.ftc.teamcode.uncledrew.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.uncledrew.hardware.DrewDrive;

@Autonomous(name="45 Degree: DriveBridgeGoRight", group="45 Degree")
@Disabled
public class FortyFiveDriveBridgeGoRight extends LinearOpMode {

    private DrewDrive robot;

    @Override
    public void runOpMode() {

        robot = new DrewDrive(this, hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.moveSide(0.9, 3000);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
