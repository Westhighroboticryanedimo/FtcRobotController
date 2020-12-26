package org.firstinspires.ftc.teamcode.uncledrew.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.uncledrew.hardware.DrewDrive;
import org.firstinspires.ftc.teamcode.uncledrew.hardware.Scoop;

@Autonomous(name="45 Degree: DrivePushPlateLeft", group="45 Degree")
@Disabled
public class FortyFiveDrivePushPlateLeft extends LinearOpMode {

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

        robot.moveSide(-0.6, -1550);

        robot.moveForward(0.6, 3270);

        scoop.scoopSetPower(0.6, 2000);

        robot.moveForward(-0.5, -4100);

        robot.turn(1, -30);

        scoop.scoop(-0.6, 50);

        robot.turn(1, 30);

        robot.moveSide(0.9, 5250);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
