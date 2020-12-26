package org.firstinspires.ftc.teamcode.uncledrew.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {

    private CRServo claw;
    private Servo gripL;
    private Servo gripR;

    private DcMotor leftElbow;
    private DcMotor rightElbow;

    private static final double ARM_POWER = 0.35;

    private static final double GRIPL_NORMAL_POSITION = 0.7;
    private static final double GRIPR_NORMAL_POSITION = 0.9;
    private static final double GRIPL_GRIP_POSITION = 1.0;
    private static final double GRIPR_GRIP_POSITION = 0.4;
    private static final int MAX_ARM_POSITION = 2750;

    private static final double CLAW_POWER = 0.002;
    private static final double CLAW_MAX = 0.75;
    private static final double CLAW_MIN = 0.65;

    private boolean autoClaw = true;

    private double clawPower;

    public Grabber(HardwareMap hwMap) {

        // Initialize left motor
        leftElbow = hwMap.get(DcMotor.class, "leftElbow");
        leftElbow.setDirection(DcMotor.Direction.REVERSE);
        leftElbow.setPower(0);
        leftElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize right motor
        rightElbow = hwMap.get(DcMotor.class, "rightElbow");
        rightElbow.setDirection(DcMotor.Direction.FORWARD);
        rightElbow.setPower(0);
        rightElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize servos
        claw = hwMap.get(CRServo.class, "claw");
        gripL = hwMap.get(Servo.class, "gripL");
        gripR = hwMap.get(Servo.class, "gripR");

    }

    public void toggleAutoClaw(boolean button) {

        if (button) autoClaw = !autoClaw;

    }

    public void moveArm(boolean buttonLift, boolean buttonDrop) {

        double armPower;
        double armPosition = leftElbow.getCurrentPosition() + rightElbow.getCurrentPosition() / 2.0;

        // Press bumpers to move arm
        if (buttonLift && armPosition > 0) {
            armPower = -ARM_POWER;
        }
        else if (buttonDrop && armPosition < MAX_ARM_POSITION) {
            armPower = ARM_POWER;
        }
        else {
            armPower = 0;
        }

        rightElbow.setPower(armPower);
        leftElbow.setPower(armPower);

    }

    public void setClawPosition(boolean buttonLift, boolean buttonDrop) {

        double armPosition = leftElbow.getCurrentPosition() + rightElbow.getCurrentPosition() / 2.0;

        if (autoClaw) {

            clawPower = CLAW_MAX - ((CLAW_MAX - CLAW_MIN) * ((armPosition - 1300) / (MAX_ARM_POSITION - 1300)));

            if (armPosition < 500) clawPower = CLAW_MAX + 0.25;

        }
        else {

            if (buttonDrop && clawPower < CLAW_MAX + 0.2)
                clawPower += CLAW_POWER;
            if (buttonLift && clawPower > CLAW_MIN)
                clawPower -= CLAW_POWER;

        }
        claw.setPower(clawPower);

    }

    public void gripStone(boolean button) {

        if (button) {

            gripL.setPosition(GRIPL_GRIP_POSITION);
            gripR.setPosition(GRIPR_GRIP_POSITION);

        }
        else {

            gripL.setPosition(GRIPL_NORMAL_POSITION);
            gripR.setPosition(GRIPR_NORMAL_POSITION);

        }

    }

}