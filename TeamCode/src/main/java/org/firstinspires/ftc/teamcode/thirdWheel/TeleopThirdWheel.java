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
    private double desiredTicks = 0.0;

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
        // invert right trigger so that unpressed is 1 and fully pressed is 0
        // math is gud
        double slow = (-1)*controller.right_trigger + 1;
        drive.drive(controller.left_stick_x*slow, controller.left_stick_y*slow, controller.right_stick_x*slow);
        // drive.togglePOV(controller.leftStickButtonOnce());
        if (controller.dpadRight()) {
            lift.override(3, -1);
            desiredTicks = lift.getEndPos();
        }
        if (controller.dpadLeft()) {
            lift.override(0, -1);
            desiredTicks = lift.getEndPos();
        }
        if (controller.rightBumperOnce()) {
            if (lift.getLevel() != 3) {
                lift.override((lift.getLevel()+1), -1);
                desiredTicks = lift.getEndPos();
            }
        }
        if (controller.leftBumperOnce()) {
            if (lift.getLevel() != 0) {
                lift.override((lift.getLevel()-1), -1);
                desiredTicks = lift.getEndPos();
            }
        }
        if (controller.A()) {
            // max doesn't want this
            // lift.inhale();
            lift.override(-1, 0);
        }
        if (controller.B()) {
            lift.override(-1, 1);
        }
        if (controller.X()) {
            lift.override(-1, 2);
        }
        if (controller.dpadUp()) {
            desiredTicks += 20;
            lift.manual(desiredTicks);
        } else if (controller.dpadDown()) {
            desiredTicks -= 20;
            lift.manual(desiredTicks);
        }
        lift.assist();
    }
}
