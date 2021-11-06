package org.firstinspires.ftc.teamcode.miscellaneous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Shooter")
public class Shooter extends OpMode {

    private DcMotor motor;

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotor.class, "motor");

    }

    @Override
    public void loop() {

        if (gamepad1.a) motor.setPower(1);
        else motor.setPower(0);

    }

}
