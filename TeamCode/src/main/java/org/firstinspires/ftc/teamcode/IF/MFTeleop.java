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
    private Servo servo;
    private DcMotor liftMotor;

    @Override
    public void init() {
        drive = new MFDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
    }
"ohnfdklsfjsafjo"
    @Override
    public void loop() {
        controller.update();
        drive.drive(-controller.left_stick_x, -controller.left_stick_y, -controller.right_stick_x);
//        telemetry.addData("gyro", gyrog)
        controller.update();
        if (controller.dpadUp()){
            liftMotor.setPower(-0.5);
        } else if (controller.dpadDown()){
            liftMotor.setPower(0.5);
        } else {
            liftMotor.setPower(0);
        }
    }
}
