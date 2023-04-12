package org.firstinspires.ftc.teamcode.fifthWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fifthWheel.command.Place;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.DriveFifthWheel;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

@TeleOp(name = "FifthWheel Red TeleOp")
public class RedTeleopFifthWheel extends OpMode {
    private DriveFifthWheel drive;
    private Place place;
    private Controller controller1;
    private Controller controller2;
    private Gyro gyro;

    private static final double MAX_ACCEL = 2.75;
    private double x = 0.0;
    private double y = 0.0;
    private double turn = 0.0;

    private int level = 0;
    private double input = 0.0;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();
    double previousTime = 0.0;
    double elapsedTime = 0.0;

    private boolean start = true;
    private boolean stacking = false;

    @Override
    public void init() {
        drive = new DriveFifthWheel(this, hardwareMap);
        place = new Place(hardwareMap, "liftLeft", "liftRight", "touch", "flipLeft", "flipRight", "grip", Gripper.Alliance.RED);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        gyro = new Gyro(hardwareMap);
    }

    @Override
    public void loop() {
        if (start) {
            place.pickup();
            start = false;
        }
        telemetry.addData("loop timer", timer.milliseconds());
        timer.reset();
        telemetry.addData("left ticks", place.getLeftPos());
        telemetry.addData("right ticks", place.getRightPos());
        // telemetry.addData("timer", place.timer.milliseconds());
        telemetry.addData("state", place.state);
        telemetry.addData("level", level);
        telemetry.addData("gyro", gyro.getAngleDegrees());
        telemetry.addData("p", place.drcb.p);
        telemetry.addData("d", place.drcb.d);
        telemetry.addData("ff", place.drcb.ff);
        telemetry.addData("output", place.drcb.output);
        telemetry.addData("total", place.drcb.total);
        telemetry.addData("level", place.drcb.level);
        telemetry.addData("setpoint", place.drcb.setpoint);
        telemetry.addData("motor power", place.drcb.liftRight.getPower());
        telemetry.addData("red", place.gripper.color.getNormalizedColors().red);
        telemetry.addData("blue", place.gripper.color.getNormalizedColors().blue);
        telemetry.update();
        controller1.update();
        controller2.update();

        elapsedTime = runtime.seconds() - previousTime;
        previousTime = runtime.seconds();
        x += Math.max(-MAX_ACCEL*elapsedTime, Math.min(controller1.left_stick_x - x, MAX_ACCEL*elapsedTime));
        y += Math.max(-MAX_ACCEL*elapsedTime, Math.min(controller1.left_stick_y - y, MAX_ACCEL*elapsedTime));
        turn += Math.max(-MAX_ACCEL*elapsedTime, Math.min(controller1.right_stick_x - turn, MAX_ACCEL*elapsedTime));
        if (controller1.right_trigger > 0.9) {
            drive.drive(x/2, y/2, turn/2);
        } else {
            drive.drive(x, y, turn);
        }

        if (controller2.AOnce()) {
            place.intake();
        } else if (controller2.XOnce()) {
            place.dropAndLower();
        } else if (controller2.dpadUpOnce()) {
            level = 3;
            place.raise(level);
        } else if (controller2.dpadRightOnce()) {
            level = 2;
            place.raise(level);
        } else if (controller2.dpadLeftOnce()) {
            level = 1;
            place.raise(level);
        } else if (controller2.dpadDownOnce()) {
            level = 0;
            place.raise(level);
        } else if (controller2.BOnce()) {
            if (stacking) {
                place.liftOffStack();
                stacking = false;
            } else {
                place.pickup();
            }
        } else if (controller2.rightBumperOnce()) {
            place.dip(true);
        } else if (controller2.leftBumperOnce()) {
            place.dip(false);
        }
//        if (controller2.leftBumperOnce()) {
//            place.goToStack();
//            stacking = true;
//        } else if (controller2.rightBumperOnce()) {
//            place.decrementStack();
//        } else
//        else {
//            place.dip(controller2.rightBumper());
//        }

//        if (controller2.left_trigger > 0) {
//            input = controller1.left_trigger*(-0.4);
//        } else if (controller2.right_trigger > 0) {
//            input = controller1.right_trigger*0.4;
//        }
        place.run(input);
    }
}
