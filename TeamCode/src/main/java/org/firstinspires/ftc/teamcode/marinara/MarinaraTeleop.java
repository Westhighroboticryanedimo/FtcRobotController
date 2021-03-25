package org.firstinspires.ftc.teamcode.marinara;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.marinara.hardware.Grabber;
import org.firstinspires.ftc.teamcode.marinara.hardware.Intake;
import org.firstinspires.ftc.teamcode.marinara.hardware.Shooter;
import org.firstinspires.ftc.teamcode.marinara.hardware.MarinaraDrive;
import org.firstinspires.ftc.teamcode.marinara.hardware.Webcam;

@TeleOp(name = "Marinara: TeleOp", group = "Marinara")
public class MarinaraTeleop extends OpMode {

    // Objects
    private Intake intake;
    private Shooter shooter;
    private Grabber grabber;
    private MarinaraDrive drive;
    private Webcam webcam;

    private Controller controller;

    @Override
    public void init() {

        intake = new Intake(this, hardwareMap);
        shooter = new Shooter(this, hardwareMap);
        grabber = new Grabber(this, hardwareMap);
        drive = new MarinaraDrive(this, hardwareMap);
        webcam = new Webcam(this, hardwareMap);

        controller = new Controller(gamepad1);

        // Unhook pop-out intake system
        intake.unhook();

        // Uncomment for telemetry values

        // intake.debug();
        shooter.debug();
        // grabber.debug();
        // drive.debug();
        // webcam.debug();

    }

    @Override
    public void loop() {

        // Controller
        controller.update();

        // Drive
        drive.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
        telemetry.addData("Voltage", drive.getVoltage(hardwareMap));
        // Drive modes
        drive.togglePOV(controller.backOnce());
        drive.toggleSlow((controller.leftStickButtonOnce() && controller.rightStickButton()) ||
                                  (controller.leftStickButton() && controller.rightStickButtonOnce()));

        // Shooter
        shooter.shoot(controller.Y(), drive.getVoltage(hardwareMap));

        // Intake
        intake.intake(controller.leftBumper(), controller.rightBumper());

        // Grabber
        grabber.grab(controller.XOnce());
        grabber.liftToDown(controller.start() && controller.dpadDownOnce());
        grabber.liftToUp(controller.start() && controller.dpadUpOnce());
        grabber.rotate(controller.BOnce());
        grabber.lift(controller.dpadUp() || grabber.getIsRaise(), controller.dpadDown() || grabber.getIsLower());
        grabber.update();

        // Webcam
        webcam.update();

        // Telemetry
        telemetry.update();

    }

}
