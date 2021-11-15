package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Intake extends BaseHardware {

    // Motor and motor power
    private DcMotor intakeMotor;
    private static final double INTAKE_POWER = 0.7;
    private static final double OUTTAKE_POWER = 1;

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
        intakeMotor = hwMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setPower(0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void intake(boolean intake, boolean reverse) {

        if (intake) intake();
        else if (reverse) reverse();
        else stop();

    }

    public void intake() {

        intakeMotor.setPower(INTAKE_POWER);

    }

    public void reverse() {

        intakeMotor.setPower(-OUTTAKE_POWER);

    }

    public void stop() {

        intakeMotor.setPower(0);

    }

}
