package org.firstinspires.ftc.teamcode.booger;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "ElijahIsSoCool")
public class ElijahIsSoCoolTeleOP extends OpMode {
    private DcMotor left, right, lift;
    private Controller controller;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotor.class,"backLeft");
        right = hardwareMap.get(DcMotor.class,"backRight");
        lift = hardwareMap.get(DcMotor.class,"frontLeft");
        controller = new Controller(gamepad1);
    }
    @Override
    public void loop() {
        if(controller.left_stick_y != 0) {
            left.setPower(controller.left_stick_y);
            right.setPower(-controller.left_stick_y);
        }
        if(controller.right_stick_y != 0) {

            left.setPower(controller.left_stick_y);
            right.setPower(controller.left_stick_y);
        }
        if(controller.dpadUp()) {
            lift.setPower(0.3);
        }
        if (controller.dpadDown()) {
            lift.setPower(-0.3);
        }
        telemetry.addData("cool:","yes");
        telemetry.update();
    }
}
