package org.firstinspires.ftc.teamcode.ultimategoal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.ultimategoal.hardware.Grabber;
import org.firstinspires.ftc.teamcode.ultimategoal.hardware.Intake;
import org.firstinspires.ftc.teamcode.ultimategoal.hardware.Shooter;
import org.firstinspires.ftc.teamcode.ultimategoal.hardware.UltimateGoalDrive;
import org.firstinspires.ftc.teamcode.ultimategoal.hardware.Webcam;

@TeleOp(name = "Ultimate Goal: TeleOp", group = "UltimateGoal")
public class UltimateGoalTeleop extends OpMode {

    // Objects
    private Intake intake;
    private Shooter shooter;
    private Grabber grabber;
    private UltimateGoalDrive drive;
    private Webcam webcam;

    private Controller controller;

    @Override
    public void init() {

        intake = new Intake(this, hardwareMap);
        shooter = new Shooter(this, hardwareMap);
        grabber = new Grabber(this, hardwareMap);
        drive = new UltimateGoalDrive(this, hardwareMap);
        webcam = new Webcam(this, hardwareMap);

        controller = new Controller(gamepad1);

        // Uncomment for telemetry values

        // intake.debug();
        shooter.debug();
        // grabber.debug();
        // drive.debug();
        webcam.debug();

    }

    @Override
    public void loop() {

        // Controller
        controller.update();

        // Drive
        drive.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
        drive.togglePOV(controller.backOnce());

        // Shooter
        shooter.shoot(controller.Y(), webcam.getDisplacement());

        // Intake
        intake.intake(controller.leftBumper(), controller.rightBumper());

        // Grabber
        grabber.grab(controller.XOnce());
        grabber.rotate(controller.BOnce());
        grabber.lift(controller.dpadUp(), controller.dpadDown());
        grabber.update();

        // Webcam
        webcam.update();

        // Telemetry
        telemetry.update();

    }

}
