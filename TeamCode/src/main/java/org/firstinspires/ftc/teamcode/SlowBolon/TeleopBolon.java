package org.firstinspires.ftc.teamcode.SlowBolon;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.freehug.Freehugdrive;


@TeleOp(name = "BOLON teleop")
public class TeleopBolon extends OpMode{

    private DriveBolon drive;
    private Controller controller;


    @Override
    public void init() {
        drive = new DriveBolon(this, hardwareMap);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        controller.update();
        drive.drive(controller.left_stick_x,controller.left_stick_y,controller.right_stick_y);
    }
}
