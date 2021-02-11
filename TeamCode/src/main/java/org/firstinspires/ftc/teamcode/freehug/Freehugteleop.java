package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "Free Hugs teleop")
public class Freehugteleop extends OpMode {
    private Freehugdrive drive;
    private IntakeFree intake;
    private Controller controller;
    @Override
    public void init() {
        drive = new Freehugdrive(this, hardwareMap);
        intake = new IntakeFree(this, hardwareMap);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        controller.update();
        drive.drive(controller.left_stick_x*0.8,-controller.left_stick_y*0.8,controller.right_stick_x);
        intake.intake(controller.B(), controller.A());
    }
}
