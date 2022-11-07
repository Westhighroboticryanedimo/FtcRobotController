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

    @Override
    public void init() {
        drive = new DriveFifthWheel(this, hardwareMap);
        place = new Place(hardwareMap, "leftMotor", "rightMotor", "flipLeft", "flipRight", "grip");
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        telemetry.update();
        controller.update();

        x.add(controller.left_stick_x);
        y.add(controller.left_stick_y);
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
            drive.drive(avgX/2.5, avgY/2.5, avgTurn/2.5);
        } else {
            drive.drive(avgX, avgY, avgTurn);
        }

        if (controller.AOnce()) {
            place.intake();
        } else if (controller.BOnce()) {
            place.pickup();
        } else if (controller.dpadUp()) {
            place.raise(3);
        } else if (controller.dpadRight()) {
            place.raise(2);
        } else if (controller.dpadLeft()) {
            place.raise(1);
        } else if (controller.dpadDown()) {
            place.raise(0);
        } else if (controller.XOnce()) {
            place.dropAndLower();
        }
        place.run();
    }
}
