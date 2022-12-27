package org.firstinspires.ftc.teamcode.MF;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.MF.subsystems.ColorCam;

@Autonomous(name = "Read/Park")

public class MFTonomous extends LinearOpMode {

    ColorCam colorCam = new ColorCam();
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;
    private Controller controller;

    private class MotorFns {

        private int getEncoders() {
            return((abs(FRDrive.getCurrentPosition()*2)+abs(BLDrive.getCurrentPosition()))/2);
        }
        private void resetEncoders() {
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
        controller = new Controller(gamepad1);
        FLDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        FRDrive = hardwareMap.get(DcMotor.class, "frontRight");
        BLDrive = hardwareMap.get(DcMotor.class, "backLeft");
        BRDrive = hardwareMap.get(DcMotor.class, "backRight");
        telemetry.addData("color", colorCam.getColor());
        telemetry.addData("hue", colorCam.getHue());
        telemetry.addData("FREncoder", FRDrive.getCurrentPosition());
        telemetry.addData("BLEncoder", BLDrive.getCurrentPosition());
        telemetry.update();
        motorFns.resetEncoders();

        while (!isStarted() && !isStopRequested()) {
            if (controller.dpadUp()) {
                colorCam.change_p1Y(-5);
            } else if (controller.dpadDown()) {
                colorCam.change_p1Y(5);
            } else if (controller.dpadRight()) {
                colorCam.change_p1X(5);
            } else if (controller.dpadLeft()) {
                colorCam.change_p1X(-5);
            } else if (controller.Y()) {
                colorCam.change_p2Y(-5);
            } else if (controller.A()) {
                colorCam.change_p1Y(5);
            } else if (controller.B()) {
                colorCam.change_p2X(5);
            } else if (controller.X()) {
                colorCam.change_p2X(-5);
            }
        }

        waitForStart();
        while (motorFns.getEncoders() < 2000) {
            motorFns.runMotors(0.25, 0.25, -0.3, -0.25);
            telemetry.update();
        }
        motorFns.stopMotors();
        motorFns.resetEncoders();
        while (motorFns.getEncoders() < 2000) {
            motorFns.runMotors(-0.25, -0.25, 0.3, 0.25);
            telemetry.update();
        }
        motorFns.stopMotors();

    }
}