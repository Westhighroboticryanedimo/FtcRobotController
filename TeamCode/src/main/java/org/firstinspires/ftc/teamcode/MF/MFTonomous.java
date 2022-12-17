package org.firstinspires.ftc.teamcode.MF;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import static java.lang.Math.abs;

@Autonomous(name = "Read/Park")

public class MFTonomous extends LinearOpMode {

    ColorCam colorCam = new ColorCam();
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;

    private class MotorFns {

        private int getEncoders() {
            return((abs(FRDrive.getCurrentPosition())+abs(BLDrive.getCurrentPosition()))/2);
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

        waitForStart();
        while (motorFns.getEncoders() < 2000) {
            motorFns.runMotors(0.5, 0.5, -0.5, -0.5);
            telemetry.update();
        }
        motorFns.stopMotors();
        motorFns.resetEncoders();
        while (FRDrive.getCurrentPosition() < 2000 && BLDrive.getCurrentPosition() > -2000) {
            motorFns.runMotors(-0.5, -0.5, 0.5, 0.5);
            telemetry.update();
        }
        motorFns.stopMotors();

    }
}