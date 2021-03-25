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

            // Move to target zone and turn
            drive.move(1, 45, 15);

            // Shoot
            shooter.shoot(0.75, drive.getVoltage(hardwareMap), intake);

            // Continue moving to target zone
            drive.move(1, 36, 15);

            // Deploy wobble goal
            drive.turn(0.6, 180);
            deployer.release();
            drive.turn(0.6, -180);

            // Grab wobble goal
            drive.move(1, 64, -163);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0.5, 12, 90);
            sleep(400);
            grabber.closeGrabber();

            // Go to target zone and deploy wobble goal
            drive.move(1, 60, 14);
            grabber.openGrabber();

            // Move away
            drive.move(1, 10, -90);

        } else if (numRingStack == 1) {

            // Move to target zone
            drive.move(1, 105, 0);

            // Deploy wobble goal
            drive.turn(0.6, 180);
            deployer.release();
            drive.turn(0.6, -180);

            // Grab wobble goal
            drive.move(1, 90.5, 180);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0.5, 17, 90);
            sleep(400);
            grabber.closeGrabber();

            // Move to target zone
            drive.move(1, 35, 10);

            // Shoot
            shooter.shoot(0.75, drive.getVoltage(hardwareMap), intake);

            // Continue moving to target zone and deploy wobble goal
            drive.move(1, 45,  -10);
            drive.move(1, 13,  -45);
            grabber.openGrabber();

            // Park on start line
            drive.move(1, 10, -90);
            drive.moveUntil(0.7, 180, drive.new ColorCommand());

        } else {

            // Move to target zone
            drive.move(1, 65, 0);

            // Shoot
            drive.move(1, 15, 90);
            shooter.shoot(0.75, drive.getVoltage(hardwareMap), intake);

            // Continue to move to target zone
            drive.move(1, 56, 14);

            // Deploy wobble goal
            drive.turn(0.6, 180);
            deployer.release();
            drive.turn(0.6, -180);

            // Grab wobble goal
            drive.move(1, 70, -160);
            drive.move(1, 35, -175);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0.5, 23, 90);
            grabber.closeGrabber();

            // Move to target zone
            drive.move(1, 22, 45);
            drive.move(1, 100, 0);
            grabber.openGrabber();

            // Park on start line
            drive.move(1, 10, -90);
            drive.moveUntil(0.7, 180, drive.new ColorCommand());

        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
