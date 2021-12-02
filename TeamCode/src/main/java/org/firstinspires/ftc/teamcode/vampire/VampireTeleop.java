package org.firstinspires.ftc.teamcode.vampire;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.vampire.hardware.Arm;
import org.firstinspires.ftc.teamcode.vampire.hardware.DuckDuckGo;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.hardware.VampireDrive;
import org.firstinspires.ftc.teamcode.vampire.hardware.Webcam;

@TeleOp(name = "VAMPIRE: TeleOp")
public class VampireTeleop extends OpMode {

    // Subsystems
    private VampireDrive drive;
    private Intake intake;
    private Arm arm;
    private DuckDuckGo spin;
    private Webcam webcam;
    private Controller controller;

    @Override
    public void init() {

        // Initialize subsystems
        drive = new VampireDrive(this, hardwareMap);
        intake = new Intake(this, hardwareMap);
        arm = new Arm(this, hardwareMap);
        spin = new DuckDuckGo(this, hardwareMap);
        webcam = new Webcam(this, hardwareMap);
        controller = new Controller(gamepad1);

        // Debug mode
        intake.debug();
        //drive.debug();
        //arm.debug();
        webcam.debug();

    }

    @Override
    public void loop() {

        controller.update();

        // Drive controls
        drive.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
        drive.togglePOV(controller.backOnce());
        drive.toggleSlow(controller.leftStickButtonOnce());

        // Other subsystem controls
        intake.intake(controller.leftBumper(), controller.rightBumper());
        spin.spin(controller.X(), controller.A());
        // arm.toggleAuto(controller.startOnce());
        arm.lift(controller.dpadUp(), controller.dpadDown());
        arm.changeStage(controller.dpadUpOnce(), controller.dpadDownOnce());

        // Update webcam values
        webcam.update();

        telemetry.update();

    }

}
