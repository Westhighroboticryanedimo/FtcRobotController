package org.firstinspires.ftc.teamcode.BoogerBoy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "Booger Boy Teleop")
public class BoogerBoyTeleop extends OpMode {

    private BoogerBoyDrive drive;
    private Controller controller;
    private DcMotor lift;
    private Servo grabby;
    private Gyro gyro;
    private int maxliftdistance;

    @Override
    public void init() {
        drive = new BoogerBoyDrive(this,hardwareMap);
        drive.setup();
        gyro = new Gyro(hardwareMap, false);
        gyro.reset();
        controller = new Controller(gamepad1);

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabby = hardwareMap.get(Servo.class, "grabby");
        //grabby = hardwareMap.get(CRServo.class, "grabby");
        maxliftdistance = -8100;
    }

    @Override
    public void loop() {
        controller.update();
        telemetry.addData(" ", "booger boy activate");

        drive.drive(controller.left_stick_x,controller.left_stick_y,controller.right_stick_x);

        if(controller.dpadUp() && lift.getCurrentPosition() > maxliftdistance) { // direction: up
            //lift.setDirection(DcMotor.Direction.FORWARD);
            telemetry.addData("","dpadup");
            lift.setPower(-1);
        } else if(controller.dpadDown() && lift.getCurrentPosition() < 0) { // direction: down
            //lift.setDirection(DcMotor.Direction.REVERSE);
            telemetry.addData("","dpaddown");
            lift.setPower(1);
        } else {
            lift.setPower(0);
        }

        if(controller.X()) {
            grabby.setPosition(0.2);
            //grabby.setDirection(CRServo.Direction.REVERSE);
            //grabby.setPower(0.5);
        }
        if(controller.Y()) {
            grabby.setPosition(1);
            //grabby.setDirection(CRServo.Direction.FORWARD);
            //grabby.setPower(0.5);
        }
        telemetry.addData("lift value:",lift.getCurrentPosition());
        telemetry.update();
    }
}
