package org.firstinspires.ftc.teamcode.terrence;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.drive.DifferentialDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Terrence: Teleop", group="Terrence")
//@Disabled
public class TerrenceSource extends OpMode {

    private DifferentialDrive drive;
    private Servo highFive;
    private Servo wave;

    private Controller controller;

    @Override
    public void init() {

        drive = new DifferentialDrive(this, hardwareMap);
        drive.debug();
        highFive = hardwareMap.get(Servo.class, "highFive");
        wave = hardwareMap.get(Servo.class, "wave");

        controller = new Controller(gamepad1);

    }

    @Override
    public void loop() {

        controller.update();

        drive.toggleArcade(controller.backOnce());
        drive.drive(controller.left_stick_y, controller.right_stick_y, controller.right_stick_x);

        if (controller.XOnce()) wave.setPosition(0.3);
        else wave.setPosition(0.7);

        if (controller.AOnce()) highFive.setPosition(0.3);
        else highFive.setPosition(0.7);

        telemetry.update();

    }

}
