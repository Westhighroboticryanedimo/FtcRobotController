package org.firstinspires.ftc.teamcode.fifthWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fifthWheel.command.Place;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.DriveFifthWheel;
import org.firstinspires.ftc.teamcode.Controller;

import java.util.ArrayList;

@TeleOp(name = "FifthWheel TeleOp")
public class TeleopFifthWheel extends OpMode {

    private DriveFifthWheel drive;
    private Place place;
    private Controller controller;

    private static final int STORE_NUM = 8;
    private ArrayList<Double> x = new ArrayList<>();
    private ArrayList<Double> y = new ArrayList<>();
    private ArrayList<Double> turn = new ArrayList<>();

    private int level = 0;

    @Override
    public void init() {
        drive = new DriveFifthWheel(this, hardwareMap);
        place = new Place(hardwareMap, "leftMotor", "rightMotor", "flipLeft", "flipRight", "grip");
        controller = new Controller(gamepad1);
        place.intake();
    }

    @Override
    public void loop() {
        telemetry.addData("timer", place.timer.milliseconds());
        telemetry.addData("helpme", place.helpme);
        telemetry.addData("state", place.state);
        telemetry.addData("level", level);
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
        } else if (controller.dpadUp()) {
            level = 3;
            place.raise(level);
        } else if (controller.dpadRight()) {
            level = 2;
            place.raise(level);
        } else if (controller.dpadLeft()) {
            level = 1;
            place.raise(level);
        } else if (controller.dpadDownOnce()) {
            if (level > 0) {
                level -= 1;
                place.raise(level);
            }
        } else if (controller.XOnce()) {
            place.dropAndLower();
        }
        place.run();
    }
}
