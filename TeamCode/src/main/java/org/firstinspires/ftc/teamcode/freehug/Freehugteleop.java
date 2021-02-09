package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Free Hugs teleop")
public class Freehugteleop extends OpMode {
    private Freehugdrive drive;
    @Override
    public void init() {
        drive = new Freehugdrive(this, hardwareMap)

    }

    @Override
    public void loop() {
        drive.drive(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);

    }
}
