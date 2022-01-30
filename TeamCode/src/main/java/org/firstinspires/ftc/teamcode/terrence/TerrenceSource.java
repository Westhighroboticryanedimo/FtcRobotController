package org.firstinspires.ftc.teamcode.terrence;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.hardware.drive.DifferentialDrive;

//@Disabled
@TeleOp(name="Terrence: Teleop", group="Terrence")

public class TerrenceSource extends OpMode {

    private DifferentialDrive drive;
    private Gyro gyro;

    private Controller controller;

    @Override
    public void init() {

        drive = new DifferentialDrive(this, hardwareMap);
        drive.debug();
        gyro = new Gyro(hardwareMap);

        controller = new Controller(gamepad1);

    }

    @Override
    public void loop() {

        controller.update();
        telemetry.addData("Gyro", gyro.getAngleRadians());
        drive.toggleArcade(controller.backOnce());
        drive.drive(controller.left_stick_y, controller.right_stick_y, controller.right_stick_x);

        telemetry.update();

    }

}
