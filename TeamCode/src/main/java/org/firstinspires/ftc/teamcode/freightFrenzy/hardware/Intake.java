package org.firstinspires.ftc.teamcode.freightFrenzy.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Intake extends BaseHardware {

    // Motor and motor power
    private DcMotor left;
    private DcMotor right;
    private static final double INTAKE_POWER = 1;

    // Teleop constructor
    public Intake(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    // Autonomous constructor
    public Intake(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    private void setup(HardwareMap hwMap) {

        // Set up motors
        left = hwMap.get(DcMotor.class, "intakeLeft");
        left.setDirection(DcMotor.Direction.FORWARD);
        left.setPower(0);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set up motors
        right = hwMap.get(DcMotor.class, "intakeLeft");
        right.setDirection(DcMotor.Direction.REVERSE);
        right.setPower(0);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void intake(boolean intake, boolean reverse) {

        if (intake) {

            left.setPower(INTAKE_POWER);
            right.setPower(INTAKE_POWER);

        } else if (reverse) {

            left.setPower(-INTAKE_POWER);
            right.setPower(-INTAKE_POWER);

        } else {

            stop();

        }

    }

    public void stop() {

        left.setPower(0);

    }

}
