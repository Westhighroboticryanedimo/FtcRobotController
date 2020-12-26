package org.firstinspires.ftc.teamcode.uncledrew.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Scoop {

    private DcMotor scoopLeft;
    private DcMotor scoopRight;

    private static final double SCOOP_POWER = 0.6;

    private LinearOpMode opmode = null;

    public Scoop(HardwareMap hwMap) {

        setupMotors(hwMap);

    }

    public Scoop(LinearOpMode opmode, HardwareMap hwMap) {

        this.opmode = opmode;

        setupMotors(hwMap);

    }

    private void setupMotors(HardwareMap hwMap) {

        // Set up left motor
        scoopLeft = hwMap.get(DcMotor.class, "scoopLeft");
        scoopLeft.setDirection(DcMotor.Direction.FORWARD);
        scoopLeft.setPower(0);
        scoopLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scoopLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scoopLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up right motor
        scoopRight = hwMap.get(DcMotor.class, "scoopRight");
        scoopRight.setDirection(DcMotor.Direction.REVERSE);
        scoopRight.setPower(0);
        scoopRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scoopRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scoopRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void scoop(double speed, int position) {

        opmode.sleep(100);

        // First set scoopLeft position
        if (position > scoopLeft.getCurrentPosition()) {

            while (opmode.opModeIsActive() && scoopLeft.getCurrentPosition() < position) {

                scoopLeft.setPower(speed);

            }

        }
        else {

            while (opmode.opModeIsActive() && scoopLeft.getCurrentPosition() > position) {

                scoopLeft.setPower(speed);

            }

        }

        // Next set scoopRight position
        if (position > scoopRight.getCurrentPosition()) {

            while (opmode.opModeIsActive() && scoopRight.getCurrentPosition() < position) {

                scoopRight.setPower(speed);

            }

        }
        else {

            while (opmode.opModeIsActive() && scoopRight.getCurrentPosition() > position) {

                scoopRight.setPower(speed);

            }

        }

    }

    public void scoop(double speed, char side, int position) {

        opmode.sleep(100);

        if (side == 'l') {

            if (position > scoopLeft.getCurrentPosition()) {

                while (opmode.opModeIsActive() && scoopLeft.getCurrentPosition() < position) {

                    scoopLeft.setPower(speed);

                }

            }
            else {

                while (opmode.opModeIsActive() && scoopLeft.getCurrentPosition() > position) {

                    scoopLeft.setPower(speed);

                }

            }

        }
        else if (side == 'r') {

            if (position > scoopRight.getCurrentPosition()) {

                while (opmode.opModeIsActive() && scoopRight.getCurrentPosition() < position) {

                    scoopRight.setPower(speed);

                }

            }
            else {

                while (opmode.opModeIsActive() && scoopRight.getCurrentPosition() > position) {

                    scoopRight.setPower(speed);

                }

            }

        }
        else
            opmode.telemetry.addLine("The side was not specified with the correct argument");

    }

    public void scoopSetPower(double speed, long time) {

        opmode.sleep(100);

        scoopLeft.setPower(speed);
        scoopRight.setPower(speed);

        opmode.sleep(time);

    }

    public void scoopSetPower(double speed, char side, long time) {

        opmode.sleep(100);

        if (side == 'r')
            scoopRight.setPower(speed);
        else if (side == 'l')
            scoopLeft.setPower(speed);
        else
            opmode.telemetry.addLine("The side was not specified with the correct argument");

        opmode.sleep(time);

    }

    public void scoopLeft(boolean buttonLift, boolean buttonDrop) {

        double scoopLPower;
        if (buttonLift) {
            scoopLPower = -SCOOP_POWER * 0.3;
        }
        else if (buttonDrop) {
            scoopLPower = SCOOP_POWER * 0.3;
        }
        else {
            scoopLPower = 0;
        }

        scoopLeft.setPower(scoopLPower);

    }

    public void scoopRight(boolean buttonLift, boolean buttonDrop) {

        double scoopRPower;
        if (buttonLift) {
            scoopRPower = -SCOOP_POWER * 0.3;
        }
        else if (buttonDrop) {
            scoopRPower = SCOOP_POWER * 0.3;
        }
        else {
            scoopRPower = 0;
        }
        scoopRight.setPower(scoopRPower);

    }

}
