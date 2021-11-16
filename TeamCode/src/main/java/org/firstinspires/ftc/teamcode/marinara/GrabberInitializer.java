package org.firstinspires.ftc.teamcode.marinara;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.marinara.hardware.Grabber;

@Autonomous(name = "Marinara Grabber Init", group = "Marinara")
@Disabled
public class GrabberInitializer extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize grabber
        Grabber grabber = new Grabber(this, hardwareMap);
        grabber.debug();
        grabber.extendRotator();
        grabber.closeGrabber();

        // Position to raise lift to
        final double LIFT_INIT_POS = 1500;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Move down the lift until limit switch is pressed
        while (opModeIsActive() && !grabber.isLimitPressed()) {

            grabber.lift(false, true);

        }

        // Reset the lift position
        grabber.resetLiftPos();

        // Move up the lift until reaching correct position
        while (opModeIsActive() && grabber.getLiftPosition() < LIFT_INIT_POS) {

            grabber.lift(true, false);

        }

        // Stop the lift
        grabber.stopLift();

        // Retract the grabber
        grabber.retractRotator();

        sleep(2000);

    }

}
