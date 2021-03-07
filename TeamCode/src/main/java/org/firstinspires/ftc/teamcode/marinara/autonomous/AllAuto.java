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

        // Check number of ring that is stacked
        if (numRingStack == 0) {

            // Move to box and turn
            drive.move(1,50,15);

            // Shoot
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 6) {

                shooter.shoot(true, new float[] {0, 0, 0});

                if (runtime.seconds() > 1)
                    intake.intake(true, false);

            }
            shooter.stopShoot();
            intake.stop();

            drive.move(1,31,15);
            drive.turn(0.6,180);

            //Turn and move back
            deployer.release();
            drive.turn(0.6,-180);
            drive.move(1,62,-168);

            //Grab wobble goal and drop wobble goal in box
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0.5,10, 90);
            sleep(400);
            grabber.closeGrabber();

            drive.move(1,65,14);
            grabber.openGrabber();

        } else if (numRingStack == 1) {

            drive.move(1,105,0);
            drive.turn(0.6,180);
            deployer.release();
            drive.turn(0.6,-180);
            drive.move(1,89,180);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0.5,16,90);
            sleep(400);
            grabber.closeGrabber();
            drive.move(1,35,5);

            // Shoot
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 6) {

                shooter.shoot(true, new float[] {0, 0, 0});

                if (runtime.seconds() > 1)
                    intake.intake(true, false);

            }
            shooter.stopShoot();
            intake.stop();

            drive.move(1,52,-8);
            grabber.openGrabber();
            drive.move(1,10,-90);
            drive.move(1,20,-180);

        } else {

            drive.moveUntil(0.7, -5, drive.new ColorCommand());
            drive.move(1, 40, 30);
            drive.turn(1, 180);
            deployer.release();
            drive.turn(1, -180);

            drive.moveUntil(0.7, -150, drive.new ColorCommand());
            drive.move(1, 50, 180);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0,16, 90);
            grabber.closeGrabber();
            drive.move(1,10,45);
            drive.moveUntil(0.7, 0, drive.new ColorCommand());
            drive.move(1,30,0);
            grabber.openGrabber();
            drive.moveUntil(0.7, 180, drive.new ColorCommand());




        }

/*
        drive.move(1, 50, 0);
        drive.move(0.5, 30, 135);
        drive.turn(0.5, 90);

 */

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
