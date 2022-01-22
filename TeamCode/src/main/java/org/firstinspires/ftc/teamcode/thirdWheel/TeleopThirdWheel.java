package org.firstinspires.ftc.teamcode.thirdWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

@TeleOp(name = "ThirdWheel TeleOp")
public class TeleopThirdWheel extends OpMode {

    private DriveThirdWheel drive;
    private Lift lift;
    private Gyro gyro;
    private Controller controller;

    @Override
    public void init() {
        drive = new DriveThirdWheel(this, hardwareMap);
        lift = new Lift(this, hardwareMap);
        gyro = new Gyro(hardwareMap, false);
        controller = new Controller(gamepad1);
        drive.togglePOV(true);
        gyro.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("gyro", gyro.getAngleDegrees());
        telemetry.addData("ticks", lift.getCurrentTicks());
        telemetry.addData("level", lift.getLevel());
        telemetry.addData("picked", lift.picked());
        telemetry.addData("state", lift.state());
        telemetry.update();
        controller.update();
        drive.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
        // drive.togglePOV(controller.leftStickButtonOnce());
        if (controller.dpadUp()) {
            lift.override(3, -1);
        }
        if (controller.dpadDown()) {
            lift.inhale();
        }
        if (controller.rightBumperOnce()) {
            if (lift.getLevel() != 3) {
                lift.override((lift.getLevel()+1), -1);
            }
        }
        if (controller.leftBumperOnce()) {
            if (lift.getLevel() != 0) {
                lift.override((lift.getLevel()-1), -1);
            }
        }
        if (controller.A()) {
            lift.override(-1, 0);
        }
        if (controller.B()) {
            lift.override(-1, 1);
        }
        if (controller.X()) {
            lift.override(-1, 2);
        }
        lift.assist();
    }
}
