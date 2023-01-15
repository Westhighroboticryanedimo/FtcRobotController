package org.firstinspires.ftc.teamcode.MF;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.MF.subsystems.ColorCam;

@Autonomous(name = "MFTonomous Right Low")

public class MFTonomousRightLow extends LinearOpMode {

    ColorCam colorCam = new ColorCam();
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;
    private Servo clawServo;
    private TouchSensor liftLimit;
    private DcMotor liftMotor;
    private DcMotor liftMotor2;

    private class MotorFns {

        private int getEncoders() {
            return((abs(FLDrive.getCurrentPosition()*2)+abs(FRDrive.getCurrentPosition()*2)+abs(BLDrive.getCurrentPosition()))/2);
        }
        private void resetEncoders() {
            FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        private void stopMotors() {
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);
        }
        private void runMotors(double FL, double BL, double FR, double BR) {
            FLDrive.setPower(FL);
            BLDrive.setPower(BL);
            FRDrive.setPower(FR);
            BRDrive.setPower(BR);
        }

    }

    private MotorFns motorFns;

    @Override public void runOpMode() throws InterruptedException {
        colorCam.cameraInit(hardwareMap);
        motorFns = new MotorFns();
        Controller controller = new Controller(gamepad1);
        FLDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        FRDrive = hardwareMap.get(DcMotor.class, "frontRight");
        BLDrive = hardwareMap.get(DcMotor.class, "backLeft");
        BRDrive = hardwareMap.get(DcMotor.class, "backRight");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        liftLimit = hardwareMap.get(TouchSensor.class, "liftLimit");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("color", colorCam.getColor());
        telemetry.addData("hue", colorCam.getHue());
        telemetry.addData("FREncoder", FRDrive.getCurrentPosition());
        telemetry.addData("BLEncoder", BLDrive.getCurrentPosition());
        telemetry.update();
        motorFns.resetEncoders();

        colorCam.set_p1Y(305);
        colorCam.set_p2Y(385);
        colorCam.set_p1X(125);
        colorCam.set_p2X(155);

        while (!isStarted() && !isStopRequested()) {
            controller.update();
            if (controller.dpadUpOnce()) {
                colorCam.change_p1Y(-5);
            } else if (controller.dpadDownOnce()) {
                colorCam.change_p1Y(5);
            } else if (controller.dpadRightOnce()) {
                colorCam.change_p1X(5);
            } else if (controller.dpadLeftOnce()) {
                colorCam.change_p1X(-5);
            } else if (controller.YOnce()) {
                colorCam.change_p2Y(-5);
            } else if (controller.AOnce()) {
                colorCam.change_p2Y(5);
            } else if (controller.BOnce()) {
                colorCam.change_p2X(5);
            } else if (controller.XOnce()) {
                colorCam.change_p2X(-5);
            } else if (controller.leftStickButtonOnce()) {
                colorCam.change_p1X(-5);
                colorCam.change_p2X(-5);
            } else if (controller.rightStickButtonOnce()) {
                colorCam.change_p1X(5);
                colorCam.change_p2X(5);
            } else if (controller.left_trigger == 1) {
                colorCam.set_sleeveType(0);
            } else if (controller.right_trigger == 1) {
                colorCam.set_sleeveType(1);
            }

            telemetry.addData("color", colorCam.getColor());
            telemetry.addData("hue", colorCam.getHue());
            telemetry.addData("p1X", colorCam.getp1X());
            telemetry.addData("p2X", colorCam.getp2X());
            telemetry.addData("p1Y", colorCam.getp1Y());
            telemetry.addData("p2Y", colorCam.getp2Y());
            telemetry.update();
        }

        waitForStart();
        int realColor = colorCam.getColor();
        telemetry.addData("Sleeve Color", realColor);
        telemetry.update();

        clawServo.setPosition(0.3);
        sleep(1000);

        while (liftMotor.getCurrentPosition() > -2200) {
            liftMotor.setPower(-1);
            liftMotor2.setPower(1);
        }
        liftMotor.setPower(0);
        liftMotor2.setPower(0);

        while (motorFns.getEncoders() < 830) {
            motorFns.runMotors(-0.27, -0.27, 0.25, 0.25);
        }
        motorFns.stopMotors();
        motorFns.resetEncoders();
        sleep(1000);

        clawServo.setPosition(0.15);
        sleep(1000);

        while (motorFns.getEncoders() < 200) {
            motorFns.runMotors(0.27, 0.27, -0.25, -0.25);
        }
        motorFns.stopMotors();
        motorFns.resetEncoders();
        sleep(1000);

        while (!liftLimit.isPressed()) {
            liftMotor.setPower(1);
            liftMotor2.setPower(-1);
        }
        liftMotor.setPower(0);
        liftMotor2.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawServo.setPosition(0);
        sleep(1000);

        while (motorFns.getEncoders() < 1300) {
                motorFns.runMotors(-0.27, 0.27, -0.25, 0.25);
            }
            motorFns.stopMotors();
            motorFns.resetEncoders();
            sleep(1000);

        while (motorFns.getEncoders() < 2700) {
            motorFns.runMotors(-0.258, -0.258, 0.25, 0.25);
        }
        motorFns.stopMotors();
        motorFns.resetEncoders();
        sleep(1000);

        if (realColor == 1) {
            while (motorFns.getEncoders() < 3100) {
                motorFns.runMotors(0.27, -0.27, 0.25, -0.25);
            }
            motorFns.stopMotors();
            motorFns.resetEncoders();
        } else if (realColor == 2) {

        } else if (realColor == 3) {
            while (motorFns.getEncoders() < 2900) {
                motorFns.runMotors(-0.27, 0.27, -0.25, 0.25);
            }
            motorFns.stopMotors();
            motorFns.resetEncoders();
        }
    }
}