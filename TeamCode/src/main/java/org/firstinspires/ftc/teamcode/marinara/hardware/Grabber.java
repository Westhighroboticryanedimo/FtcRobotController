package org.firstinspires.ftc.teamcode.marinara.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private static final double GRABBER_POS_START = 0.1;
    private static final double GRABBER_POS_END = 1;
    private static final double GRABBER_SPEED = 0.01;
    private static final double ROTATOR_POS_START = 1;
    private static final double ROTATOR_POS_END = 0;
    private static final double LIFT_MAX_POS = 1500;
    private static final double LIFT_POWER = 0.4;

    // Variable for grabber
    private boolean isGrabberOpen = false;

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
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Setup rotator
        rotator = hwMap.get(Servo.class, "rotator");
        rotator.setPosition(ROTATOR_POS_START);

        // Setup grabber
        grabber = hwMap.get(Servo.class, "grabber");
        grabber.setPosition(GRABBER_POS_START);

        // Setup sensor
        touch = hwMap.get(TouchSensor.class, "touch");

    }

    private void resetLiftPos() {

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void lift(boolean raise, boolean lower) {

        if (raise) {

            // Raise grabber as long as the encoder value doesn't exceed LIFT_MAX_POS
            if (lift.getCurrentPosition() < LIFT_MAX_POS) {

                lift.setPower(LIFT_POWER);

            }

        } else if (lower) {

            // Lower grabber as long as the touch sensor doesn't detect the lift
            if (!touch.isPressed()) {

                lift.setPower(-LIFT_POWER);

            }

        } else {

            lift.setPower(0);

        }

    }

    public void grab(boolean toggle) {

        if (toggle) {

            // Toggle isGrabberOpen
            isGrabberOpen = !isGrabberOpen;

        }

        if (!isGrabberOpen) {

            if (grabber.getPosition() > GRABBER_POS_START) {

                // If grabber is open, then close it slowly
                grabber.setPosition(grabber.getPosition() - GRABBER_SPEED);

            } else {

                grabber.setPosition(GRABBER_POS_START);

            }

        } else {

            // If grabber is closed, open it
            grabber.setPosition(GRABBER_POS_END);

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
