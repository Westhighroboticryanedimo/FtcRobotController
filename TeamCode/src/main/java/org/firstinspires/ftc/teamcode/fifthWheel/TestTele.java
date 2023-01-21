package org.firstinspires.ftc.teamcode.fifthWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.DRCB;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.DriveFifthWheel;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.Controller;

import java.util.ArrayList;

@TeleOp(name = "FifthWheel Test TeleOp")
public class TestTele extends OpMode {

    private DriveFifthWheel drive;
    private Gripper gripper;
    private DRCB drcb;
    private Gyro gyro;
    private Controller controller;

    private static final int STORE_NUM = 8;
    private ArrayList<Double> x = new ArrayList<>();
    private ArrayList<Double> y = new ArrayList<>();
    private ArrayList<Double> turn = new ArrayList<>();

    @Override
    public void init() {
        drive = new DriveFifthWheel(this, hardwareMap);
        gripper = new Gripper(hardwareMap, "flipLeft", "flipRight", "gripLeft", "gripRight");
//        gripper.setLevel(0);
        drcb = new DRCB(hardwareMap, "leftMotor", "rightMotor", "touch");
        gyro = new Gyro(hardwareMap, false);
        controller = new Controller(gamepad1);
        drcb.setLevel(0);
        // gripper.setLevel(0);
        gyro.reset();
        gripper.flipLeft.turnToAngle(20);
        gripper.flipRight.turnToAngle(20);
        gripper.gripLeft.turnToAngle(20);
        gripper.gripRight.turnToAngle(20);
    }

    @Override
    public void loop() {
        telemetry.addData("gyro", gyro.getAngleDegrees());
//        telemetry.addData("leftPos", gripper.leftPos());
//        telemetry.addData("rightPos", gripper.rightPos());
        telemetry.addData("left drcb", drcb.getCurrentLeftTicks());
        telemetry.addData("right drcb", drcb.getCurrentRightTicks());
        telemetry.addData("p", drcb.p);
        telemetry.addData("d", drcb.d);
        telemetry.addData("ff", drcb.ff);
        telemetry.addData("output", drcb.output);
        telemetry.addData("total", drcb.total);
        telemetry.addData("level", drcb.level);
        telemetry.addData("setpoint", drcb.setpoint);
        telemetry.addData("drcb i gain", drcb.i);
        telemetry.addData("flipLeft angle", gripper.flipLeft.getAngle());
        telemetry.addData("flipRight angle", gripper.flipRight.getAngle());
        telemetry.addData("gripLeft angle", gripper.gripLeft.getAngle());
        telemetry.addData("gripRight angle", gripper.gripRight.getAngle());
        telemetry.addData("isPressed", drcb.touch.isPressed());
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
            // drive.drive(avgX/2.5, avgY/2.5, avgTurn/2.5);
        } else {
            // drive.drive(avgX, avgY, avgTurn);
        }

        if (controller.leftBumperOnce()) {
            // drcb.p -= 0.001;
            // drcb.updatePID();
//            drcb.justFeedforward = true;
            gripper.moveDown();
        } else if (controller.rightBumperOnce()) {
            // drcb.p += 0.001;
            // drcb.updatePID();
//            drcb.justFeedforward = false;
            gripper.moveUp();
        }
        if (controller.XOnce()) {
            drcb.d -= 0.001;
            drcb.updatePID();
        } else if (controller.BOnce()) {
            drcb.d += 0.001;
            drcb.updatePID();
        }
        if (controller.dpadUpOnce()) {
            drcb.setLevel(3);
        } else if (controller.dpadRightOnce()) {
            drcb.setLevel(2);
        } else if (controller.dpadLeftOnce()) {
            drcb.setLevel(1);
        } else if (controller.dpadDownOnce()) {
            drcb.setLevel(0);
        }
        if (controller.AOnce()) {
            // drcb.useMotionProfile = false;
//            drive.updatePID();
            gripper.moveClose();
        } else if (controller.YOnce()) {
            // drcb.useMotionProfile = true;
            gripper.moveOpen();
        }
         drcb.run(0);
    }
}
