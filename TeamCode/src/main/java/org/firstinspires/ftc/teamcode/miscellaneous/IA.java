package org.firstinspires.ftc.teamcode.miscellaneous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "IA")
public class IA extends OpMode {

    private DcMotor motor;

    @Override
    public void init() {

        motor = hardwareMap.get(DcMotor.class, "motor");

    }

    @Override
    public void loop() {

        motor.setPower(0.5);

    }

}
