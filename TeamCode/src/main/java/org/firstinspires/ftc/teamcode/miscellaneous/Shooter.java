package org.firstinspires.ftc.teamcode.miscellaneous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "Shooter")
@Disabled
public class Shooter extends OpMode {

    private DcMotor motor;
    private double power = 1;
    private boolean isShooting = false;
    private Controller controller;

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotor.class, "motor");
        controller = new Controller(gamepad1);

    }

    @Override
    public void loop() {

        controller.update();

        if (controller.AOnce()) isShooting = !isShooting;
        if (controller.dpadUpOnce() && power <= 0.95) power += 0.1;
        if (controller.dpadDownOnce() && power >= 0.05) power -= 0.1;
        if (isShooting) motor.setPower(power);
        else motor.setPower(0);

        telemetry.addData("Power: ", power);

    }

}

