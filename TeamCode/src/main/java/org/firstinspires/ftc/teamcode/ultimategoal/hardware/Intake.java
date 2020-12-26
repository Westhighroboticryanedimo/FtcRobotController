package org.firstinspires.ftc.teamcode.ultimategoal.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Intake extends BaseHardware {

    // Motor and motor power
    private DcMotor intakeMotor;
    private static final double INTAKE_POWER = 1.0;

    // Teleop constructor
    public Intake(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setupMotor(hwMap);

    }

    // Autonomous constructor
    public Intake(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setupMotor(hwMap);

    }

    private void setupMotor(HardwareMap hwMap) {

        intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setPower(0);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void intake(boolean intake, boolean reverse) {

        if (intake) {

            intakeMotor.setPower(INTAKE_POWER);

        } else if (reverse) {

            intakeMotor.setPower(-INTAKE_POWER);

        } else {

            stop();

        }

    }

    public void stop() {

        intakeMotor.setPower(0);

    }

    @Override
    public void update() {



    }

}