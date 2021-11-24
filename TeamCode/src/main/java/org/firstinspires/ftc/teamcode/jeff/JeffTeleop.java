package org.firstinspires.ftc.teamcode.jeff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;
import org.firstinspires.ftc.teamcode.marinara.hardware.Grabber;
import org.firstinspires.ftc.teamcode.marinara.hardware.Intake;
import org.firstinspires.ftc.teamcode.marinara.hardware.MarinaraDrive;
import org.firstinspires.ftc.teamcode.marinara.hardware.Shooter;
import org.firstinspires.ftc.teamcode.marinara.hardware.Webcam;

@TeleOp(name = "Jeffy boi")
@Disabled
public class JeffTeleop extends OpMode {

    // Objects
    private JeffDrive drive;

    // Controller object
    private Controller controller;

    @Override
    public void init() {

        drive = new JeffDrive(this, hardwareMap);
        drive.debug();
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {

        // Controller
        controller.update();

        // Drive
        //drive.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);

        drive.highFive(controller.YOnce());
        drive.wave(controller.AOnce());

        telemetry.addData("Wave timer", drive.waveTimer);
        telemetry.addData("Five timer", drive.fiveTimer);
        //telemetry.addData("red", drive.color.red());
        //telemetry.addData("green", drive.color.green());
        //telemetry.addData("blue", drive.color.blue());

        //drive.checkColor();

        // Telemetry
        telemetry.update();

    }

}
