package org.firstinspires.ftc.teamcode.marinara.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.marinara.hardware.Grabber;
import org.firstinspires.ftc.teamcode.marinara.hardware.Intake;
import org.firstinspires.ftc.teamcode.marinara.hardware.MarinaraDrive;
import org.firstinspires.ftc.teamcode.marinara.hardware.Shooter;
import org.firstinspires.ftc.teamcode.marinara.hardware.Webcam;

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

        drive.move(1, 80, 0);
        drive.turn(1, -45);

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
        drive.move(1, 70, 25);

        // Check number of ring that is stacked
        if (numRingStack == 0) {

            // Move to target zone A and drop wobble goal
            drive.move(1, 20, 45);
            grabber.openGrabber();

            // Retract grabber
            drive.move(0.5, 2, -90);
            grabber.closeGrabber();
            grabber.retractRotator();

        } else if (numRingStack == 1) {

            // Move to target zone B and drop wobble goal
            drive.move(1, 30, 0);
            grabber.openGrabber();

            // Retract grabber
            drive.move(0.5, 2, -90);
            grabber.closeGrabber();
            grabber.retractRotator();

            // Move to starting line
            drive.move(1, 20, -180);

        } else {

            // Move to target zone C and drop wobble goal
            drive.move(1, 60, 25);
            grabber.openGrabber();

            // Retract grabber
            drive.move(0.5, 2, -90);
            grabber.closeGrabber();
            grabber.retractRotator();

            // Move to starting line
            drive.move(1, 50, -180);

        }

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }

}
