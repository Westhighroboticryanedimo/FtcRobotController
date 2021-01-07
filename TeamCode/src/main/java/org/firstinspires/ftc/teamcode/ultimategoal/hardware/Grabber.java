package org.firstinspires.ftc.teamcode.ultimategoal.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Grabber extends BaseHardware {

    // Objects
    private DcMotor lift = null;
    private Servo rotator = null;
    private Servo grabber = null;
    private TouchSensor touch = null;

    // Final variables
    private static final double GRABBER_POS_START = 0;
    private static final double GRABBER_POS_END = 1;
    private static final double ROTATOR_POS_START = 0;
    private static final double ROTATOR_POS_END = 1;
    private static final double LIFT_MIN_POS = -100;
    private static final double LIFT_MAX_POS = 1500;
    private static final double LIFT_POWER = 0.4;

    // Teleop constructor
    public Grabber(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    // Autonomous constructor
    public Grabber(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    private void setup(HardwareMap hwMap) {

        // Setup lift motor
        lift = hwMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setPower(0);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Setup rotator
        rotator = hwMap.get(Servo.class, "rotator");
        rotator.setPosition(0);

        // Setup grabber
        grabber = hwMap.get(Servo.class, "grabber");
        grabber.setPosition(0);

        // Setup sensor
        touch = hwMap.get(TouchSensor.class, "touch");

    }

    private void resetLiftPos() {

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void lift(boolean raise, boolean lower) {

        if (raise) {

            if (lift.getCurrentPosition() < LIFT_MAX_POS) {

                lift.setPower(LIFT_POWER);

            }

        } else if (lower) {

            if (lift.getCurrentPosition() > LIFT_MIN_POS) {

                lift.setPower(-LIFT_POWER);

            }

        } else {

            lift.setPower(0);

        }

    }

    public void grab(boolean toggle) {

        if (toggle) {

            if (grabber.getPosition() == GRABBER_POS_START) {

                grabber.setPosition(GRABBER_POS_END);

            } else {

                grabber.setPosition(GRABBER_POS_START);

            }

        }

    }

    public void rotate(boolean button) {

        if (button) {

            if (rotator.getPosition() == ROTATOR_POS_START) {

                rotator.setPosition(ROTATOR_POS_END);

            } else {

                rotator.setPosition(ROTATOR_POS_START);

            }

        }

    }

    public void update() {

        print("Lift position: ", lift.getCurrentPosition());
        print("isPressed: ", touch.isPressed());

        if (touch.isPressed()) {

            resetLiftPos();

        }

    }

}
