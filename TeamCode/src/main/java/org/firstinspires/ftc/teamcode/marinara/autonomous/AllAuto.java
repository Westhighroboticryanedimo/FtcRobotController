package org.firstinspires.ftc.teamcode.marinara.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.marinara.hardware.Grabber;
import org.firstinspires.ftc.teamcode.marinara.hardware.Intake;
import org.firstinspires.ftc.teamcode.marinara.hardware.MarinaraDrive;
import org.firstinspires.ftc.teamcode.marinara.hardware.Shooter;
import org.firstinspires.ftc.teamcode.marinara.hardware.Webcam;
import org.firstinspires.ftc.teamcode.marinara.hardware.WobbleDeployer;

@Autonomous(name = "Marinara All Auto", group = "Marinara")
//@Disabled
public class AllAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Get all hardware
        MarinaraDrive drive = new MarinaraDrive(this, hardwareMap);
        Grabber grabber = new Grabber(this, hardwareMap);
        Intake intake = new Intake(this, hardwareMap);
        Shooter shooter = new Shooter(this, hardwareMap);
        Webcam webcam = new Webcam(this, hardwareMap);
        WobbleDeployer deployer = new WobbleDeployer(this, hardwareMap);

        // Elapsed time for timed motion
        ElapsedTime runtime = new ElapsedTime();

        // Uncomment to debug and print information of the hardware
        //drive.debug();
        //grabber.debug();
        //intake.debug();
        //shooter.debug();
        //webcam.debug();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Distance right", drive.getDistanceRight());
            telemetry.addData("Distance back", drive.getDistanceBack());
            telemetry.addData("Back check", drive.new DistanceBackCommand(10).check());
            telemetry.addData("Right check", drive.new DistanceRightCommand(10).check());
            telemetry.update();

        }

/*
        // Get how many rings are stacked
        int numRingStack = 0;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {

            numRingStack = webcam.getNumRings();

            telemetry.addData("Num rings", webcam.getNumRings());
            telemetry.addData("Ring position", webcam.getInternalRingNum());
            telemetry.update();

        }

        // Check number of ring that is stacked
        if (numRingStack == 0) {

            // Move to target zone and turn
            drive.move(0.7, 39, 15);

            // Shoot
            shooter.shoot(0.72, drive.getVoltage(hardwareMap), intake);

            // Continue moving to target zone
            drive.move(0.7, 28, 10);

            // Deploy wobble goal
            drive.turn(0.3, 180);
            deployer.release();
            drive.turn(0.3, -180);

            // Grab wobble goal
            drive.move(0.7, 48, -160);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0.3, 20, 90);
            sleep(400);
            grabber.closeGrabber();

            // Go to target zone and deploy wobble goal
            drive.move(0.7, 54, 14);
            grabber.openGrabber();

            // Move away
            drive.move(0.7, 10, -88);

        } else if (numRingStack == 1) {

            // Move to target zone
            drive.move(0.7, 105, 0);

            // Deploy wobble goal
            drive.turn(0.3, 180);
            deployer.release();
            drive.turn(0.3, -180);

            // Grab wobble goal
            drive.move(0.7, 86, 180);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0.3, 16, 90);
            sleep(400);
            grabber.closeGrabber();

            // Move to target zone
            drive.move(0.7, 35, 0);

            // Shoot
            shooter.shoot(0.75, drive.getVoltage(hardwareMap), intake);

            // Continue moving to target zone and deploy wobble goal
            drive.move(0.7, 52,  -8);
            grabber.openGrabber();

            // Park on start line
            drive.move(0.7, 10, -90);
            drive.moveUntil(0.5, 160, drive.new ColorCommand());

        } else {

            // Move to target zone
            drive.move(0.7, 65, 0);

            // Shoot
            drive.move(0.7, 15, 90);
            shooter.shoot(0.735, drive.getVoltage(hardwareMap), intake);

            // Continue to move to target zone
            drive.move(0.7, 50, 10);

            // Deploy wobble goal
            drive.turn(0.3, 180);
            deployer.release();
            drive.turn(0.3, -180);

            // Grab wobble goal
            drive.move(0.7, 40, -160);
            drive.move(0.7, 48, -180);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0.3, 21, 90);
            grabber.closeGrabber();

            // Move to target zone
            drive.move(0.7, 23, 45);
            drive.move(0.7, 93, 0);
            grabber.openGrabber();

            // Park on start line
            drive.move(0.7, 10, -90);
            drive.moveUntil(0.5, 160, drive.new ColorCommand());

        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

 */

    }

}
