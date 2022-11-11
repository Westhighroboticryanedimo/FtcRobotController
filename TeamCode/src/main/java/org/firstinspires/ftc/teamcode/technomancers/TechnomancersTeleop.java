package org.firstinspires.ftc.teamcode.technomancers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.technomancers.TechnomancersDrive;

@TeleOp(name = "Technomancers TeleOp")
public class TechnomancersTeleop extends OpMode {

    private TechnomancersDrive drive;
    private Controller controller;
    private Servo servo;
    private DcMotor dcMotor;

    @Override
    public void init() {
        drive = new TechnomancersDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        servo = hardwareMap.get(Servo.class,  "servo");
        dcMotor = hardwareMap.get(DcMotor.class,  "dcMotor");
        servo.setPosition(0.6);
    }

    @Override
    public void loop() {
        controller.update();
        drive.drive(-controller.left_stick_x, -controller.left_stick_y, -controller.right_stick_x);
        controller.update();
        if (controller.A()) {
            servo.setPosition(0.5);
        }
        if (controller.B()) {
            servo.setPosition(0.3);
        }
        if (controller.leftBumper()){
            dcMotor.setPower(0.5);
        } else if (controller.rightBumper()){
            dcMotor.setPower(-0.5);
        } else {
            dcMotor.setPower(0);
        }
    }

}