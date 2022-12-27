package org.firstinspires.ftc.teamcode.fifthWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.fifthWheel.command.Place;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.DriveFifthWheel;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

@TeleOp(name = "FifthWheel TeleOp")
public class TeleopFifthWheel extends OpMode {
    private DriveFifthWheel drive;
    private Place place;
    private Controller controller;
    private Gyro gyro;

    private static final double MAX_ACCEL = 2.75;
    private double x = 0.0;
    private double y = 0.0;
    private double turn = 0.0;

    private int level = 0;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0.0;
    double elapsedTime = 0.0;

    private boolean start = true;

    @Override
    public void init() {
        drive = new DriveFifthWheel(this, hardwareMap);
        place = new Place(hardwareMap, "leftMotor", "rightMotor", "touch", "flipLeft", "flipRight", "gripLeft", "gripRight");
        controller = new Controller(gamepad1);
        gyro = new Gyro(hardwareMap);
    }

    @Override
    public void loop() {
        if (start) {
            place.intake();
            start = false;
        }
        telemetry.addData("loop timer", timer.milliseconds());
        timer.reset();
        telemetry.addData("left ticks", place.getLeftPos());
        telemetry.addData("right ticks", place.getRightPos());
//        telemetry.addData("timer", place.timer.milliseconds());
        telemetry.addData("helpme", place.helpme);
        telemetry.addData("state", place.state);
        telemetry.addData("level", level);
        telemetry.addData("gyro", gyro.getAngleDegrees());
        telemetry.addData("left drcb", place.drcb.getCurrentLeftTicks());
        telemetry.addData("right drcb", place.drcb.getCurrentRightTicks());
        telemetry.addData("p", place.drcb.p);
        telemetry.addData("d", place.drcb.d);
        telemetry.addData("ff", place.drcb.ff);
        telemetry.addData("output", place.drcb.output);
        telemetry.addData("total", place.drcb.total);
        telemetry.addData("level", place.drcb.level);
        telemetry.addData("setpoint", place.drcb.setpoint);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("turn", turn);
        telemetry.update();
        controller.update();

        elapsedTime = runtime.seconds() - previousTime;
        previousTime = runtime.seconds();
        x += Math.max(-MAX_ACCEL*elapsedTime, Math.min(controller.left_stick_x - x, MAX_ACCEL*elapsedTime));
        y += Math.max(-MAX_ACCEL*elapsedTime, Math.min(controller.left_stick_y - y, MAX_ACCEL*elapsedTime));
        turn += Math.max(-MAX_ACCEL*elapsedTime, Math.min(controller.right_stick_x - turn, MAX_ACCEL*elapsedTime));
        drive.drive(x, y, turn);

        if (controller.AOnce()) {
            place.intake();
        } else if (controller.BOnce()) {
            place.pickup();
        } else if (controller.dpadUpOnce()) {
            level = 3;
            place.raise(level);
        } else if (controller.dpadRightOnce()) {
            level = 2;
            place.raise(level);
        } else if (controller.dpadLeftOnce()) {
            level = 1;
            place.raise(level);
        } else if (controller.dpadDownOnce()) {
            level = 0;
            place.raise(level);
        } else if (controller.XOnce()) {
            place.dropAndLower();
        }
        place.run();
    }
}
