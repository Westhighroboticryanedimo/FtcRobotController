package org.firstinspires.ftc.teamcode.Wingman;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.WizardBot.Subsystems.IntakeFSM;

import org.firstinspires.ftc.teamcode.Controller;

import org.firstinspires.ftc.teamcode.Wingman.subsystems.ColorCam;

@Disabled
@Autonomous(name = "MFTonomous Parking")

public class MFTonomousPark extends LinearOpMode {

    ColorCam colorCam = new ColorCam();
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;
    private IntakeFSM intakeFSM;

    private class MotorFns {

        private int getEncoders() {
            return((abs(FLDrive.getCurrentPosition())+abs(FRDrive.getCurrentPosition())+abs(BLDrive.getCurrentPosition())+abs(BRDrive.getCurrentPosition()))/4);
        }
        private void resetEncoders() {
            FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        telemetry.addData("color", colorCam.getColor());
        telemetry.addData("hue", colorCam.getHue());
        telemetry.addData("FREncoder", FRDrive.getCurrentPosition());
        telemetry.addData("BLEncoder", BLDrive.getCurrentPosition());
        telemetry.update();
        motorFns.resetEncoders();

        intakeFSM = new IntakeFSM();
        intakeFSM.intakeInit(hardwareMap);

        colorCam.set_p1Y(380);
        colorCam.set_p2Y(475);

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
        intakeFSM.startIntakeFSM();

        if (realColor == 1 || realColor == 2) {
            while (motorFns.getEncoders() < 1250) {
                motorFns.runMotors(0.27, 0.27, -0.27, -0.27);
                BRDrive.setPower(-0.27);
                intakeFSM.intake();
                telemetry.addData("FRont LEft Encoder", FLDrive.getCurrentPosition());
                telemetry.addData("FRont Right Encoder", FRDrive.getCurrentPosition());
                telemetry.addData("Vack LEft Encoder", BLDrive.getCurrentPosition());
                telemetry.addData("Bak Right Encoder", BRDrive.getCurrentPosition());
                telemetry.update();
            }
        } else {
            while (motorFns.getEncoders() < 1050) {
                motorFns.runMotors(0.27, 0.27, -0.27, -0.27);
                BRDrive.setPower(-0.27);
                intakeFSM.intake();
                telemetry.addData("FRont LEft Encoder", FLDrive.getCurrentPosition());
                telemetry.addData("FRont Right Encoder", FRDrive.getCurrentPosition());
                telemetry.addData("Vack LEft Encoder", BLDrive.getCurrentPosition());
                telemetry.addData("Bak Right Encoder", BRDrive.getCurrentPosition());
                telemetry.update();
            }
        }

        motorFns.stopMotors();
        motorFns.resetEncoders();
        sleep(1000);
        if (realColor == 1) {
            while (motorFns.getEncoders() < 1150) {
                motorFns.runMotors(-0.3, 0.3, -0.3, 0.3);
                telemetry.addData("FRont LEft Encoder", FLDrive.getCurrentPosition());
                telemetry.addData("FRont Right Encoder", FRDrive.getCurrentPosition());
                telemetry.addData("Vack LEft Encoder", BLDrive.getCurrentPosition());
                telemetry.addData("Bak Right Encoder", BRDrive.getCurrentPosition());
                telemetry.update();
            }
            motorFns.stopMotors();
            motorFns.resetEncoders();
        } else if (realColor == 2) {

        } else if (realColor == 3) {
            while (motorFns.getEncoders() < 1250) {
                motorFns.runMotors(0.3, -0.3, 0.3, -0.3);
                telemetry.addData("FRont LEft Encoder", FLDrive.getCurrentPosition());
                telemetry.addData("FRont Right Encoder", FRDrive.getCurrentPosition());
                telemetry.addData("Vack LEft Encoder", BLDrive.getCurrentPosition());
                telemetry.addData("Bak Right Encoder", BRDrive.getCurrentPosition());
                telemetry.update();
            }
            motorFns.stopMotors();
            motorFns.resetEncoders();
        }
    }
}