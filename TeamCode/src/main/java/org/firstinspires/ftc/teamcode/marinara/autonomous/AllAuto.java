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

        // Get how many rings are stacked
        int numRingStack = 0;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {

            numRingStack = webcam.getNumRings();

            telemetry.addData("Num rings", webcam.getNumRings());
            telemetry.addData("Ring position", webcam.getInternalRingNum());
            telemetry.update();

        }

        // Move to initial position
        drive.move(1, 72, 0);

        // Check number of ring that is stacked
        if (numRingStack == 0) {

            // Move to target zone A and drop wobble goal
            drive.move(1, 28, 90);
            drive.turn(0.5, 180);
            deployer.release();
            sleep(1000);
            drive.turn(0.5, -180);

            // Move to wobble goal
            drive.move(1, 55, -165);
            grabber.openGrabber();
            grabber.extendRotator();

            // Move right and grab wobble goal
            drive.move(0.5, 14, 90);
            sleep(500);
            grabber.closeGrabber();

            // Lift the grabber
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {

                grabber.lift(true, false);

            }
            grabber.stopLift();

            // Move to target
            drive.move(1, 68, 13);

            // Deploy wobble goal
            grabber.openGrabber();

        } else if (numRingStack == 1) {

            // Move to target zone B and drop wobble goal
            drive.move(1, 22, 0);
            drive.turn(0.5, 180);
            deployer.release();
            sleep(1000);
            drive.turn(0.5, -180);

            // Move to wobble goal
            drive.move(1, 55, 180);
            grabber.openGrabber();
            grabber.extendRotator();

            // Move right and grab wobble goal
            drive.move(0.5, 14, 90);
            sleep(500);
            grabber.closeGrabber();

            // Lift the grabber
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {

                grabber.lift(true, false);

            }
            grabber.stopLift();

            // Move to target
            drive.move(1, 68, -20);

            // Deploy wobble goal
            grabber.openGrabber();

        } else {

            // Move to target zone C and drop wobble goal
            drive.move(1, 35, 30);
            drive.turn(0.5, 180);
            deployer.release();
            sleep(1000);
            drive.turn(0.5, -180);

            // Move to wobble goal
            drive.move(1, 90, -170);
            grabber.openGrabber();
            grabber.extendRotator();

            // Move right and grab wobble goal
            drive.move(0.5, 14, 90);
            sleep(500);
            grabber.closeGrabber();

            // Lift the grabber
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1) {

                grabber.lift(true, false);

            }
            grabber.stopLift();

            // Move to target
            drive.move(1, 80, 5);

            // Deploy wobble goal
            grabber.openGrabber();

        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
