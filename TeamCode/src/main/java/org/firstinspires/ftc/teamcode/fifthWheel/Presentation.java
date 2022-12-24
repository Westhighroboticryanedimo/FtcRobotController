package org.firstinspires.ftc.teamcode.fifthWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.DRCB;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.DriveFifthWheel;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.Controller;

import java.util.ArrayList;

@TeleOp(name = "FifthWheel Presentation")
public class Presentation extends OpMode {

    private Gripper gripper;
    private DRCB drcb;
    private Gyro gyro;
    private Controller controller;

    @Override
    public void init() {
        gripper = new Gripper(hardwareMap, "flipLeft", "flipRight", "gripLeft", "gripRight");
        drcb = new DRCB(hardwareMap, "leftMotor", "rightMotor", "touch");
        gyro = new Gyro(hardwareMap, false);
        controller = new Controller(gamepad1);
        drcb.setLevel(0);
        gripper.setLevel(0);
        gyro.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("gyro", gyro.getAngleDegrees());
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
        telemetry.addData("grip angle", gripper.gripLeft.getAngle());
        telemetry.update();
        controller.update();

        if (controller.leftBumperOnce()) {
           drcb.justFeedforward = true;
        } else if (controller.rightBumperOnce()) {
           drcb.justFeedforward = false;
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
            drcb.useMotionProfile = false;
        } else if (controller.YOnce()) {
            drcb.useMotionProfile = true;
        }
        drcb.run();
    }
}

