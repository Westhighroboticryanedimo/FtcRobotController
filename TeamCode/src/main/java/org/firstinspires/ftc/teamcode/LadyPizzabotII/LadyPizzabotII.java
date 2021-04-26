package org.firstinspires.ftc.teamcode.LadyPizzabotII;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="LadyPizzabotII Program")
public class LadyPizzabotII extends OpMode {

    DcMotor left;
    DcMotor right;

    @Override
    public void init() {

        //Set variables, objects, etc
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);



    }
    @Override
    public void loop() {

        double turnPower = gamepad1.right_stick_x;
        double forwardPower= -gamepad1.left_stick_y;

        double leftPower = forwardPower + turnPower;
        double rightPower = forwardPower - turnPower;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max == 0)  {

            max = 1;

        }
        left.setPower(leftPower / max);
        right.setPower(rightPower / max);

    }

}

