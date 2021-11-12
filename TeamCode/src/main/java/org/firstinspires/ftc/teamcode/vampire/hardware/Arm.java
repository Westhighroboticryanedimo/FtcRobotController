package org.firstinspires.ftc.teamcode.vampire.hardware;

import android.text.method.Touch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Arm extends BaseHardware {

    // Objects
    private DcMotor liftMotor;
    private DcMotor angleMotor;
    private TouchSensor armTouch;
    private TouchSensor angleTouch;

    // Constants
    private static final double LIFT_POWER = 0.8;
    private static final double LOWER_POWER = 0.2;
    private static final double ANGLE_POWER = 0.3;
    private static final int[] ARM_STAGES = { -100, 1000, 2000, 3000 };
    private static final int[] ANGLE_STAGES = { -100, 500, 1000, 2000 };

    // Mutable variables
    private int stage = 0;
    private boolean isAuto = false;

    // PID
    private PIDController pidArm = new PIDController(0.1, 0.1, 0.1);
    private PIDController pidAngle = new PIDController(0.1, 0.1, 0.1);

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

        // Set up PID
        pidArm.setInputRange(0, ARM_STAGES[3]);
        pidArm.setOutputRange(0, LIFT_POWER);
        pidArm.setTolerance(1);
        pidAngle.setInputRange(0, ARM_STAGES[3]);
        pidAngle.setOutputRange(0, LIFT_POWER);
        pidAngle.setTolerance(1);

        // Set up lift motor
        liftMotor = hwMap.get(DcMotor.class, "lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set up angle motor
        angleMotor = hwMap.get(DcMotor.class, "angle");
        angleMotor.setDirection(DcMotor.Direction.FORWARD);
        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleMotor.setPower(0);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set up touch sensors
        armTouch = hwMap.get(TouchSensor.class, "armTouch");
        angleTouch = hwMap.get(TouchSensor.class, "angleTouch");

    }

    public void toggleAuto(boolean button) {

        if (button) isAuto = !isAuto;

    }

    public void lift(boolean up, boolean down) {

        // Manually move arm
        if (!isAuto) {

            if (up) liftMotor.setPower(LIFT_POWER);
            else if (down) liftMotor.setPower(-LOWER_POWER);
            else liftMotor.setPower(0);

        }

    }

    public void angle(boolean up, boolean down) {

        // Manually move angle
        if (!isAuto) {

            if (up) angleMotor.setPower(ANGLE_POWER);
            else if (down) angleMotor.setPower(-ANGLE_POWER);
            else angleMotor.setPower(0);

        }

    }

    public void setLift(int s) {

        stage = s;
        pidArm.reset();
        pidArm.enable();
        pidArm.setSetpoint(ARM_STAGES[stage]);
        pidArm.setTolerance(1);
        pidAngle.reset();
        pidAngle.enable();
        pidAngle.setSetpoint(ANGLE_STAGES[stage]);
        pidAngle.setTolerance(1);

        do {

            // Telemetry
            print("Position Arm", liftMotor.getCurrentPosition());
            print("Position Angle", angleMotor.getCurrentPosition());
            linearOpMode.telemetry.update();

            // Power the motors
            liftMotor.setPower(pidArm.performPID(liftMotor.getCurrentPosition()));
            angleMotor.setPower(pidAngle.performPID(angleMotor.getCurrentPosition()));

        } while ((!pidArm.onTarget() || !pidAngle.onTarget()) ||
                (stage == 0 && (!armTouch.isPressed() || !angleTouch.isPressed())));

        if (stage == 0) {

            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }

    }

    public void changeStage(boolean up, boolean down) {

        print("Touch Arm", armTouch.isPressed());
        print("Touch Angle", angleTouch.isPressed());
        if (isAuto) {

            // Move up and down stage
            if (up && stage != 3) stage++;
            if (down && stage != 0) stage--;

            // Set PID
            pidArm.setSetpoint(ARM_STAGES[stage]);
            pidAngle.setSetpoint(ANGLE_STAGES[stage]);

            // If angle is touching touch sensor
            if (angleTouch.isPressed() && stage == 0) {

                pidAngle.disable();
                angleMotor.setPower(0);
                angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            } else {

                pidAngle.enable();
                angleMotor.setPower(pidAngle.performPID(angleMotor.getCurrentPosition()));
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

            // If lift is touching touch sensor
            if (armTouch.isPressed() && stage == 0) {

                pidArm.disable();
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            } else {

                pidArm.enable();
                liftMotor.setPower(pidArm.performPID(liftMotor.getCurrentPosition()));
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

        }

    }

}