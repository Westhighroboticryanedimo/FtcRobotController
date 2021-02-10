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
    private static final double GRABBER_POS_GRAB = 1;
    private static final double GRABBER_POS_OPEN = 0.6;
    private static final double ROTATOR_POS_EXT = 0.9;
    private static final double ROTATOR_POS_RETRACT = 0.4;
    private static final double ROTATOR_ABLE_LIFT_MIN_POS = 1500;
    private static final double SERVO_THRESHOLD = 0.01;
    private static final double LIFT_MAX_POS = 5800;
    private static final double LIFT_POWER = 0.6;

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
        stopLift();
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Setup rotator
        rotator = hwMap.get(Servo.class, "rotator");
        retractRotator();

        // Setup grabber
        grabber = hwMap.get(Servo.class, "grabber");
        closeGrabber();

        // Setup sensor
        touch = hwMap.get(TouchSensor.class, "touch");

    }

    public void resetLiftPos() {

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void lift(boolean raise, boolean lower) {

        print("Lift position: ", getLiftPosition());

        if (raise) {

            // Raise grabber as long as the encoder value doesn't exceed LIFT_MAX_POS
            if (getLiftPosition() < LIFT_MAX_POS) {

                lift.setPower(LIFT_POWER);

            } else {

                stopLift();

            }

        } else if (lower) {

            // Lower grabber as long as the touch sensor doesn't detect the lift
            if (!isLimitPressed()) {

                lift.setPower(-LIFT_POWER);

            } else {

                stopLift();

            }

        } else {

            stopLift();

        }

    }

    public void grab(boolean toggle) {

        print("Grab Position: ", grabber.getPosition());

        if (toggle) {

            if (grabber.getPosition() > GRABBER_POS_GRAB - SERVO_THRESHOLD &&
                grabber.getPosition() < GRABBER_POS_GRAB + SERVO_THRESHOLD) {

                // If grabber is closed and rotator is extended, open it
                if (rotator.getPosition() > ROTATOR_POS_EXT - SERVO_THRESHOLD &&
                    rotator.getPosition() < ROTATOR_POS_EXT + SERVO_THRESHOLD) openGrabber();

            } else {

                // Else, close it
                closeGrabber();

            }

        }

    }

    public void closeGrabber() {

        grabber.setPosition(GRABBER_POS_GRAB);

    }

    public void openGrabber() {

        grabber.setPosition(GRABBER_POS_OPEN);

    }

    public void rotate(boolean button) {

        print("Rotator Position:", rotator.getPosition());

        if (button) {

            if (rotator.getPosition() > ROTATOR_POS_EXT - SERVO_THRESHOLD &&
                rotator.getPosition() < ROTATOR_POS_EXT + SERVO_THRESHOLD) {

                if (getLiftPosition() > ROTATOR_ABLE_LIFT_MIN_POS) {

                    // Retract rotator and close the grabber
                    retractRotator();
                    closeGrabber();

                }

            } else {

                // Extend the rotator
                extendRotator();

            }

        }

    }

    public void extendRotator() {

        rotator.setPosition(ROTATOR_POS_EXT);

    }

    public void retractRotator() {

        rotator.setPosition(ROTATOR_POS_RETRACT);

    }

    public void stopLift() {

        lift.setPower(0);

    }

    public boolean isLimitPressed() {

        return touch.isPressed();

    }

    public double getLiftPosition() {

        return lift.getCurrentPosition();

    }

    public void update() {

        print("isPressed: ", isLimitPressed());

        if (isLimitPressed()) {

            resetLiftPos();

        }

    }

}
