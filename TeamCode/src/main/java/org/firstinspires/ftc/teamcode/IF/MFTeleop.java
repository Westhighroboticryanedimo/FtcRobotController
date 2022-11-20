package org.firstinspires.ftc.teamcode.IF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.IF.MFDrive;

@TeleOp(name = "MF TeleOp")
public class MFTeleop extends OpMode {

    private MFDrive drive;
    private Controller controller;
    private Servo clawServo;
    private DcMotor liftMotor;

    @Override
    public void init() {
        drive = new MFDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {
        controller.update();
        drive.drive(-controller.left_stick_x, -controller.left_stick_y, -controller.right_stick_x);
//        telemetry.addData("gyro", gyrog)
        controller.update();
        if (controller.dpadDown() && liftMotor.getCurrentPosition() > 0) {
            liftMotor.setPower(0);
        } else if (controller.dpadUp()){
            liftMotor.setPower(-0.5);
        } else if (controller.dpadDown()){
            liftMotor.setPower(0.5);
        } else {
            liftMotor.setPower(0);
        }
        if (controller.BOnce()){
            clawServo.setPosition(0.65);
        }
        if (controller.AOnce()){
            clawServo.setPosition(0);
        }
    }
}
