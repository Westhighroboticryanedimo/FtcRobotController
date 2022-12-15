package org.firstinspires.ftc.teamcode.reee.fifthWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.reee.fifthWheel.command.Place;
import org.firstinspires.ftc.teamcode.reee.fifthWheel.subsystem.DriveFifthWheel;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name = "FifthWheel TeleOp")
public class TeleopFifthWheel extends OpMode {
 df
    private DriveFifthWheel drive;
    private Place place;
    private Controller controller;
    private Gyro gyro;

    private static final int STORE_NUM = 8;
    private ArrayList<Double> x = new ArrayList<>();
    private ArrayList<Double> y = new ArrayList<>();
    private ArrayList<Double> turn = new ArrayList<>();

    private int level = 0;

    ElapsedTime timer = new ElapsedTime();

    private boolean start = true;

    @Override
    public void init() {
        drive = new DriveFifthWheel(this, hardwareMap);
        place = new Place(hardwareMap, "leftMotor", "rightMotor", "flipLeft", "flipRight", "grip");
        controller = new Controller(gamepad1);
        gyro = new Gyro(hardwareMap);
    }

    @Override
    public void loop() {
        if (start) {
            place.intake();
            start = false;
        }
        telemetry.addData("timer", timer.milliseconds());
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
        telemetry.update();
        controller.update();

        x.add(controller.left_stick_x);
        y.add(-controller.left_stick_y);
        turn.add(controller.right_stick_x);

        // Remove
        if (x.size() > STORE_NUM) x.remove(0);
        if (y.size() > STORE_NUM) y.remove(0);
        if (turn.size() > STORE_NUM) turn.remove(0);

        double avgX = 0;
        for (int i = 0; i < x.size(); i++) avgX += x.get(i);
        avgX /= x.size();

        double avgY = 0;
        for (int i = 0; i < y.size(); i++) avgY += y.get(i);
        avgY /= y.size();

        double avgTurn = 0;
        for (int i = 0; i < turn.size(); i++) avgTurn += turn.get(i);
        avgTurn /= turn.size();

        if (controller.left_trigger > 0) {
            drive.drive(avgX/2, avgY/2, avgTurn/2);
        } else {
            drive.drive(avgX, avgY, avgTurn);
        }

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
        } else if (controller.right_trigger > 0.8) {
            place.drcb.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            place.drcb.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            place.drcb.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            place.drcb.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (controller.rightBumper()) {
            place.drcb.leftMotor.setPower(-0.01);
            place.drcb.rightMotor.setPower(-0.01);
        } else {
            place.run();
        }
    }
}
