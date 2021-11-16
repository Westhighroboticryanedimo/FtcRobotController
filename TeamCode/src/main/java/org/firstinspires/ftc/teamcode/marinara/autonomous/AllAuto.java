package org.firstinspires.ftc.teamcode.marinara.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.marinara.hardware.Grabber;
import org.firstinspires.ftc.teamcode.marinara.hardware.Intake;
import org.firstinspires.ftc.teamcode.marinara.hardware.MarinaraDrive;
import org.firstinspires.ftc.teamcode.marinara.hardware.Shooter;
import org.firstinspires.ftc.teamcode.marinara.hardware.Webcam;
import org.firstinspires.ftc.teamcode.marinara.hardware.WobbleDeployer;

@Autonomous(name = "Marinara All Auto", group = "Marinara")
@Disabled
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
        drive.debug();
        //grabber.debug();
        //intake.debug();
        //shooter.debug();
        //webcam.debug();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

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
            drive.move(0.5, 68, 10);

            // Shoot
            shooter.shoot(drive.getVoltage(hardwareMap), intake);

            // Continue moving to target zone
            drive.move(0.7, 15, 45);

            // Deploy wobble goal
            drive.turn(0.3, 180);
            deployer.release();
            drive.turn(0.3, -180);

            // Grab wobble goal
            drive.moveUntilDistanceBack(0.5, -170, 16);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.moveUntilDistanceRight(0.3, 90, 28);
            grabber.closeGrabber();
            sleep(400);

            // Go to target zone and deploy wobble goal
            drive.moveUntil(0.5, -2, drive.new ColorCommand());
            grabber.openGrabber();

        } else if (numRingStack == 1) {

            // Move to target zone
            drive.move(0.7, 100, -4);

            // Deploy wobble goal
            drive.turn(0.4, 180);
            deployer.release();
            drive.turn(0.4, -180);

            // Grab wobble goal
            drive.move(1, 85, 180);
            drive.moveUntilDistanceBack(0.4, -180, 14);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.moveUntilDistanceRight(0.3, 90, 27);
            grabber.closeGrabber();
            sleep(400);

            // Move to target zone
            drive.move(0.5, 10, -22);
            drive.move(0.5, 18, 10);

            // Shoot
            shooter.shoot(drive.getVoltage(hardwareMap), intake);

            // Continue moving to target zone and deploy wobble goal
            drive.move(0.5, 30, -8);
            grabber.openGrabber();

            // Park on start line
            drive.move(0.7, 10, -90);
            drive.moveUntil(0.5, 160, drive.new ColorCommand());

        } else {

            // Move to target zone
            drive.move(0.7, 66, -7);

            // Shoot
            drive.move(0.7, 20, 95);
            shooter.shoot(drive.getVoltage(hardwareMap), intake);

            // Continue to move to target zone
            drive.move(1, 67, 7);

            // Deploy wobble goal
            drive.turn(0.4, 180);
            deployer.release();
            drive.turn(0.4, -180);

            // Park on start line
            drive.move(0.7, 10, -90);
            drive.moveUntil(0.5, 160, drive.new ColorCommand());

/*

            // Move back
            drive.move(0.8, 110, -175);

            // Grab wobble goal
            drive.moveUntilDistanceBack(0.3, 175, 16);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0.5, 32, 90);
            grabber.closeGrabber();
            sleep(400);

            // Move to target zone
            drive.move(0.7, 20, 45);
            drive.move(0.7, 92, -13);
            grabber.openGrabber();

 */

        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
