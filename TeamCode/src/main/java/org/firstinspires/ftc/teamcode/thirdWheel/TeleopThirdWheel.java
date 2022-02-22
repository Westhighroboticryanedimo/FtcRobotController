package org.firstinspires.ftc.teamcode.thirdWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

import java.util.ArrayList;

@TeleOp(name = "ThirdWheel TeleOp")
public class TeleopThirdWheel extends OpMode {

    private DriveThirdWheel drive;
    private Lift lift;
    private Gyro gyro;
    private Controller controller;
    private double desiredTicks = 0.0;

    private static final int STORE_NUM = 8;
    private ArrayList<Double> x = new ArrayList<>();
    private ArrayList<Double> y = new ArrayList<>();
    private ArrayList<Double> turn = new ArrayList<>();

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
        // this is kinda weird ngl, a pain to use
        // double slow = (-1)*controller.right_trigger + 1;

        x.add(controller1.left_stick_x);
        y.add(controller1.left_stick_y);
        turn.add(controller1.right_stick_x);

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

        drive.drive(avgX*0.75, avgY*0.75, avgTurn*0.75);
        // drive.togglePOV(controller.leftStickButtonOnce());
        if (controller.dpadUp()) {
            lift.override(3, -1);
            desiredTicks = lift.getEndPos();
        }
        if (controller.dpadDown()) {
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
//        if (controller.dpadUp()) {
//            desiredTicks += 20;
//            lift.manual(desiredTicks);
//        } else if (controller.dpadDown()) {
//            desiredTicks -= 20;
//            lift.manual(desiredTicks);
//        }
        lift.assist();
    }
}
