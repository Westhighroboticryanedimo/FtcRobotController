package org.firstinspires.ftc.teamcode.fifthWheel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.DRCB;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.DriveFifthWheel;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.Gripper;
import org.firstinspires.ftc.teamcode.fifthWheel.command.Place;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.Controller;

import java.util.ArrayList;

@TeleOp(name = "FifthWheel Presentation")
public class Presentation extends OpMode {

    private Place place;
    private Gyro gyro;
    private Controller controller;

    @Override
    public void init() {
        place = new Place(hardwareMap, "leftMotor", "rightMotor", "touch", "flipLeft", "flipRight", "gripLeft", "gripRight");
        gyro = new Gyro(hardwareMap, false);
        controller = new Controller(gamepad1);
        gyro.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("gyro", gyro.getAngleDegrees());
        telemetry.update();
        controller.update();

        if (controller.leftBumperOnce()) {
           place.drcb.justFeedforward = true;
        } else if (controller.rightBumperOnce()) {
           place.drcb.justFeedforward = false;
        }
        if (controller.AOnce()) {
            place.wave();
        }
        place.run(0);
    }
}

