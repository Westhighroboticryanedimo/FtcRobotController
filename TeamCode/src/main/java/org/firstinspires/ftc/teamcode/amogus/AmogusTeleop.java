package org.firstinspires.ftc.teamcode.amogus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "Amogus TeleOp")
public class AmogusTeleop extends OpMode {
    private Controller controller;
    private Servo servo;

    @Override
    public void init() {
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        if (controller.A()) {
            servo.setPosition(0);
        }
        if (controller.B()) {
            servo.setPosition(1);
        }
        if (controller.A()) {
            servo.setPosition(0);
        }
        // later
        if (controller.right_trigger > 0) {
            servo.setPosition(1);
        } else {
            servo.setPosition(0);
        }
    }
}