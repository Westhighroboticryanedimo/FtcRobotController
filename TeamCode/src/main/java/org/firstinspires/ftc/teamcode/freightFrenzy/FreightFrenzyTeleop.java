package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.freightFrenzy.hardware.Intake;

@TeleOp(name = "Freight Frenzy: TeleOp", group = "FF")
public class FreightFrenzyTeleop extends OpMode {

    private Intake intake;
    private Controller controller;

    @Override
    public void init() {

        intake = new Intake(this, hardwareMap);
        controller = new Controller(gamepad1);

    }

    @Override
    public void loop() {

        controller.update();
        intake.intake(controller.leftBumper(), controller.rightBumper());

        telemetry.update();

    }

}
