package org.firstinspires.ftc.teamcode.marinara.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Intake extends BaseHardware {

    // Motor and motor power
    private DcMotor intakeMotor;
    private static final double MOTOR_INTAKE_POWER = 1;
    private static final double SERVO_INTAKE_POWER = 1;

    // Servo for hook
    private Servo hook;
    private static final double HOOKED_POS = 0.5;
    private static final double HOOK_FREE_POS = 1;

    // CR Servo to intake
    private CRServo intakeServo;

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
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set up servos
        intakeServo = hwMap.get(CRServo.class, "pop_out");
        intakeServo.setDirection(DcMotor.Direction.REVERSE);
        intakeServo.setPower(0);
        hook = hwMap.get(Servo.class, "hook");

    }

    public void intake(boolean intake, boolean reverse) {

        if (intake) {

            intakeMotor.setPower(MOTOR_INTAKE_POWER);
            intakeServo.setPower(SERVO_INTAKE_POWER);

        } else if (reverse) {

            intakeMotor.setPower(-MOTOR_INTAKE_POWER);
            intakeServo.setPower(-SERVO_INTAKE_POWER);

        } else {

            stop();

        }

    }

    public void hook() {

        hook.setPosition(HOOKED_POS);

    }

    public void unhook() {

        hook.setPosition(HOOK_FREE_POS);

    }

    public void stop() {

        intakeMotor.setPower(0);
        intakeServo.setPower(0);

    }

}