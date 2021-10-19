package org.firstinspires.ftc.teamcode.thirdWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;


@TeleOp(name = "ThirdWheel TeleOp")
public class TeleopThirdWheel extends OpMode {

    private DriveThirdWheel drive;
    private Controller controller;

    @Override
    public void init() {
        drive = new DriveThirdWheel(this, hardwareMap);
        controller = new Controller(gamepad1);
        drive.togglePOV(true);
    }

    @Override
    public void loop() {
        controller.update();
        drive.drive(controller.left_stick_x,controller.left_stick_y,controller.right_stick_x);
        drive.togglePOV(controller.leftStickButtonOnce());
    }
}
