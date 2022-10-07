package org.firstinspires.ftc.teamcode.BoogerBoy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "Booger Boy Teleop")
public class BoogerBoyTeleop extends OpMode {

    private BoogerBoyDrive drive;
    private Controller controller;
    private DcMotor lift;
    //private Gyro gyro;
    //private Servo servo_claw;

    @Override
    public void init() {
        drive = new BoogerBoyDrive(this,hardwareMap);
        drive.setup();
        //gyro = new Gyro(hardwareMap, false);
        //gyro.reset();
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        controller.update();
        telemetry.addData(" ", "booger boy activate");

        drive.drive(controller.left_stick_x,controller.left_stick_y,controller.right_stick_x);




        telemetry.update();
    }
}
