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
            drive.move(0.8, 45, 15);

            // Shoot
            shooter.shoot(0.8, drive.getVoltage(hardwareMap), intake);

            // Continue moving to target zone
            drive.move(0.8, 23, 15);

            // Deploy wobble goal
            drive.turn(0.3, 180);
            deployer.release();
            drive.turn(0.3, -180);

            // Grab wobble goal
            drive.move(0.8, 48, -163);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0.3, 12, 90);
            sleep(400);
            grabber.closeGrabber();

            // Go to target zone and deploy wobble goal
            drive.move(0.8, 52, 14);
            grabber.openGrabber();

            // Move away
            drive.move(0.8, 10, -90);

        } else if (numRingStack == 1) {

            // Move to target zone
            drive.move(0.8, 105, 0);

            // Deploy wobble goal
            drive.turn(0.3, 180);
            deployer.release();
            drive.turn(0.3, -180);

            // Grab wobble goal
            drive.move(0.8, 90.5, 180);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0.3, 17, 90);
            sleep(400);
            grabber.closeGrabber();

            // Move to target zone
            drive.move(0.8, 35, 10);

            // Shoot
            shooter.shoot(0.75, drive.getVoltage(hardwareMap), intake);

            // Continue moving to target zone and deploy wobble goal
            drive.move(0.8, 45,  -10);
            drive.move(0.8, 13,  -45);
            grabber.openGrabber();

            // Park on start line
            drive.move(0.8, 10, -90);
            drive.moveUntil(0.3, 180, drive.new ColorCommand());

        } else {

            // Move to target zone
            drive.move(0.8, 65, 0);

            // Shoot
            drive.move(0.8, 15, 90);
            shooter.shoot(0.75, drive.getVoltage(hardwareMap), intake);

            // Continue to move to target zone
            drive.move(0.8, 56, 14);

            // Deploy wobble goal
            drive.turn(0.3, 180);
            deployer.release();
            drive.turn(0.3, -180);

            // Grab wobble goal
            drive.move(0.8, 70, -160);
            drive.move(0.8, 35, -175);
            grabber.openGrabber();
            grabber.extendRotator();
            drive.move(0.3, 23, 90);
            grabber.closeGrabber();

            // Move to target zone
            drive.move(0.8, 22, 45);
            drive.move(0.8, 100, 0);
            grabber.openGrabber();

            // Park on start line
            drive.move(0.8, 10, -90);
            drive.moveUntil(0.3, 180, drive.new ColorCommand());

        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
