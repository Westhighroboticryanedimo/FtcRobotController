package org.firstinspires.ftc.teamcode.uncledrew;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.uncledrew.hardware.DrewDrive;
import org.firstinspires.ftc.teamcode.uncledrew.hardware.Grabber;
import org.firstinspires.ftc.teamcode.uncledrew.hardware.Scoop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Uncle Drew: TeleOp", group = "45 Degree")
@Disabled
public class DrewTeleop extends OpMode {

    // Hardware
    private DrewDrive drive;
    private Grabber grabber;
    private Scoop scoop;
    private Controller controller1;

    @Override
    public void init() {

        drive = new DrewDrive(this, hardwareMap);
        grabber = new Grabber(hardwareMap);
        scoop = new Scoop(hardwareMap);
        controller1 = new Controller(gamepad1);

    }

    @Override
    public void loop() {

        // Update controller
        controller1.update();

        // Toggle pov and field oriented mode
        drive.togglePOV(controller1.backOnce());

        // Drive the robot
        drive.drive(controller1.left_stick_x, controller1.left_stick_y, controller1.right_stick_x);

        // Lift and Drop the arm
        grabber.moveArm(controller1.rightBumper(), controller1.leftBumper());

        // Toggle auto-claw
        grabber.toggleAutoClaw(controller1.startOnce());

        // Set claw position
        grabber.setClawPosition(controller1.Y(), controller1.A());

        // Grip stone
        grabber.gripStone(controller1.X());

        // Press dpad to move scoop
        scoop.scoopRight(controller1.dpadUp(), controller1.dpadDown());
        scoop.scoopLeft(controller1.dpadRight(), controller1.dpadLeft());

        telemetry.update();

    }

}