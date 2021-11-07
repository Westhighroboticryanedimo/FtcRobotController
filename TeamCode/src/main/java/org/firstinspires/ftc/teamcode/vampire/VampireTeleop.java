package org.firstinspires.ftc.teamcode.vampire;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.miscellaneous.Shooter;
import org.firstinspires.ftc.teamcode.vampire.hardware.Arm;
import org.firstinspires.ftc.teamcode.vampire.hardware.DuckDuckSpin;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.hardware.VampireDrive;

@TeleOp(name = "VAMPIRE: TeleOp")
public class VampireTeleop extends OpMode {

    // Subsystems
    private VampireDrive drive;
    private Intake intake;
    private Arm arm;
    private DuckDuckSpin spin;
    private Controller controller;

    @Override
    public void init() {

        // Initialize subsystems
        drive = new VampireDrive(this, hardwareMap);
        intake = new Intake(this, hardwareMap);
        arm = new Arm(this, hardwareMap);
        spin = new DuckDuckSpin(this, hardwareMap);
        controller = new Controller(gamepad1);

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
        spin.spin(controller.leftBumper());
        arm.lift(controller.dpadUp(), controller.dpadDown());
        arm.angle(controller.dpadRight(), controller.dpadLeft());

        telemetry.update();

    }

}
