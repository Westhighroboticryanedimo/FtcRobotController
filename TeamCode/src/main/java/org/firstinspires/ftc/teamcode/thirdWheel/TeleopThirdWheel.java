package org.firstinspires.ftc.teamcode.thirdWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware;

@TeleOp(name = "ThirdWheel TeleOp")
public class TeleopThirdWheel extends OpMode {

    private DriveThirdWheel drive;
    private Gyro gyro;
    private Controller controller;

    @Override
    public void init() {
        drive = new DriveThirdWheel(this, hardwareMap);
        gyro = new Gyro(hardwareMap, false);
        controller = new Controller(gamepad1);
        drive.togglePOV(true);
        gyro.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("gyro", gyro.getAngleDegrees());
        telemetry.update();
        controller.update();
        drive.drive(controller.left_stick_x,controller.left_stick_y,controller.right_stick_x);
        drive.togglePOV(controller.leftStickButtonOnce());
    }
}
