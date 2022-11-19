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
        servo.setPosition(0.4);
    }

    @Override
    public void loop() {
        controller.update();
        drive.drive(-controller.left_stick_x/2, -controller.left_stick_y/2, -controller.right_stick_x/2);
        controller.update();
        if (controller.A()) {
            servo.setPosition(0.4);
        }
        if (controller.B()) {
            servo.setPosition(0);
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