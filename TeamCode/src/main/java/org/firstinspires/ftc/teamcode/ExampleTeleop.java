package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Example TeleOp Program", group="Example")
public class ExampleTeleop extends OpMode {

    // Create a new ExampleHardware object called drive
    private ExampleHardware drive;

    @Override
    public void init() {

        // Actually set drive to the ExapleHardware class with hardwareMap passed to it
        drive = new ExampleHardware(hardwareMap);

    }

    @Override
    public void loop() {

        // Call the drive function repeatedly
        drive.drive(gamepad1.left_stick_y, gamepad1.right_stick_x);

    }

}
