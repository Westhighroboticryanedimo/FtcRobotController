package org.firstinspires.ftc.teamcode.thirdWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.LinearSlide;

@TeleOp(name = "ThirdWheel TeleOp")
public class TeleopThirdWheel extends OpMode {

    private DriveThirdWheel drive;
    private LinearSlide linearSlide;
    private Gyro gyro;
    private Controller controller;
    private int level = 0;

    @Override
    public void init() {
        drive = new DriveThirdWheel(this, hardwareMap);
        linearSlide = new LinearSlide(this, hardwareMap);
        gyro = new Gyro(hardwareMap, false);
        controller = new Controller(gamepad1);
        drive.togglePOV(true);
        gyro.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("gyro", gyro.getAngleDegrees());
        telemetry.addData("ticks", linearSlide.getCurrentTicks());
        telemetry.addData("level", linearSlide.getLevel());
        telemetry.update();
        controller.update();
        drive.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
        drive.togglePOV(controller.leftStickButtonOnce());
        if (controller.rightBumper()) {
            if (level < 3) {
                level += 1;
                linearSlide.setLevel(level);
            }
        }
        if (controller.leftBumper()) {
            if (level > 0) {
                level -= 1;
                linearSlide.setLevel(level);
            }
        }
    }
}
