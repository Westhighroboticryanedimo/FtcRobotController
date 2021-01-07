package org.firstinspires.ftc.teamcode.uncledrew.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.uncledrew.hardware.DrewDrive;
import org.firstinspires.ftc.teamcode.uncledrew.hardware.Scoop;

@Autonomous(name="45 Degree: DrivePushPlateRight", group="45 Degree")
@Disabled
public class FortyFiveDrivePushPlateRight extends LinearOpMode {

    private DrewDrive robot;
    private Scoop scoop;

    @Override
    public void runOpMode() {

        robot = new DrewDrive(this, hardwareMap);
        scoop = new Scoop(this, hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.move(0.6, 12, 90);

        robot.move(0.6, 25, 0);

        scoop.scoopSetPower(0.6, 2000) ;

        robot.move(0.5, 30, 180);

        scoop.scoop(-0.6, 50);

        robot.move(0.9, 40, -90);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
