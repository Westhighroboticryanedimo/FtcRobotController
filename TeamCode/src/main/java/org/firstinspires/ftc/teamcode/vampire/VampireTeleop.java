package org.firstinspires.ftc.teamcode.vampire;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.vampire.hardware.Arm;
import org.firstinspires.ftc.teamcode.vampire.hardware.DuckDuckGo;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.hardware.TapeArm;
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
    //private MiniArm miniArm;
    private TapeArm tapeArm;
    private Controller controller1;
    private Controller controller2;

    @Override
    public void init() {

        // Initialize subsystems
        drive = new VampireDrive(this, hardwareMap);
        intake = new Intake(this, hardwareMap);
        arm = new Arm(this, hardwareMap);
        spin = new DuckDuckGo(this, hardwareMap);
        //miniArm = new MiniArm(this, hardwareMap);
        webcam = new Webcam(this, hardwareMap);
        tapeArm = new TapeArm(this, hardwareMap);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        // Debug mode
        intake.debug();
        //drive.debug();
        //arm.debug();
        //miniArm.debug();
        webcam.debug();
        tapeArm.debug();

    }

    @Override
    public void loop() {

        // Update controller values
        controller1.update();
        controller2.update();

        // Drive controls
        drive.drive(controller1.left_stick_x, controller1.left_stick_y, controller1.right_stick_x);
        // drive.togglePOV(controller.backOnce());
        // drive.toggleSlow(controller.leftStickButtonOnce());

        // Other subsystem controls
        intake.intake(controller1.leftBumper(), controller1.rightBumper());
        spin.spin(controller1.A(), controller1.X());
        // arm.toggleAuto(controller.startOnce());
        arm.lift(controller1.dpadUp(), controller1.dpadDown());
        arm.changeStage(controller1.dpadUpOnce(), controller1.dpadDownOnce());

        // Mini arm which is not used anymore
        //miniArm.moveArm(controller.YOnce());
        //miniArm.moveClaw(controller.BOnce());
        //miniArm.continuousMoveArm(controller.dpadRight(), controller.dpadLeft());

        // The tape measure...
        tapeArm.horzMove(controller2.dpadRight(), controller2.dpadLeft());
        tapeArm.vertMove(controller2.dpadUp(), controller2.dpadDown());
        tapeArm.roll(controller2.leftBumper(), controller2.rightBumper());

        // Update webcam values
        webcam.update();

        telemetry.update();

    }

}
