package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Arm extends BaseHardware {

    // Motor and motor power
    private DcMotor liftMotor;
    private DcMotor angleMotor;
    private static final double LIFT_POWER = 0.5;
    private static final double LOWER_POWER = 0.2;
    private static final double ANGLE_POWER = 0.3;

    // Teleop constructor
    public Arm(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    // Autonomous constructor
    public Arm(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    private void setup(HardwareMap hwMap) {

        // Set up motors
        liftMotor = hwMap.get(DcMotor.class, "lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set up motors
        angleMotor = hwMap.get(DcMotor.class, "angle");
        angleMotor.setDirection(DcMotor.Direction.FORWARD);
        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleMotor.setPower(0);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void lift(boolean up, boolean down) {

        if (up) liftMotor.setPower(LIFT_POWER);
        else if (down) liftMotor.setPower(-LOWER_POWER);
        else liftMotor.setPower(0);

    }

    public void angle(boolean up, boolean down) {

        if (up) angleMotor.setPower(ANGLE_POWER);
        else if (down) angleMotor.setPower(-ANGLE_POWER);
        else angleMotor.setPower(0);

    }

}