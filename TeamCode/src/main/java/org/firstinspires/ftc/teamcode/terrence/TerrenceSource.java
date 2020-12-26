package org.firstinspires.ftc.teamcode.terrence;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.drive.DifferentialDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="Terrence: Teleop", group="Terrence")
// @Disabled
public class TerrenceSource extends OpMode {

    private DifferentialDrive drive;

    private Controller controller1;

    @Override
    public void init() {

        drive = new DifferentialDrive(this, hardwareMap);
        drive.debug();

        controller1 = new Controller(gamepad1);

    }

    @Override
    public void loop() {

        controller1.update();

        drive.toggleArcade(controller1.backOnce());
        drive.drive(controller1.left_stick_y, controller1.right_stick_y, controller1.right_stick_x);
        drive.update();

        telemetry.update();

    }

}
