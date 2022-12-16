package org.firstinspires.ftc.teamcode.MF;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Read/Park")

public class MFTonomous extends LinearOpMode {

    ColorCam colorCam = new ColorCam();
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;

    @Override public void runOpMode() throws InterruptedException {
        colorCam.cameraInit(hardwareMap);
        FLDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        FRDrive = hardwareMap.get(DcMotor.class, "frontRight");
        BLDrive = hardwareMap.get(DcMotor.class, "backLeft");
        BRDrive = hardwareMap.get(DcMotor.class, "backRight");
        telemetry.addData("color", colorCam.getColor());
        telemetry.addData("hue", colorCam.getHue());
        telemetry.addData("FREncoder", FRDrive.getCurrentPosition());
        telemetry.addData("BLEncoder", BLDrive.getCurrentPosition());
        telemetry.update();
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (FRDrive.getCurrentPosition() > -2000 && BLDrive.getCurrentPosition() < 2000) {
            FLDrive.setPower(0.5);
            FRDrive.setPower(-0.5);
            BLDrive.setPower(0.5);
            BRDrive.setPower(-0.5);
            telemetry.update();
        }
        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (FRDrive.getCurrentPosition() < 2000 && BLDrive.getCurrentPosition() > -2000) {
            FLDrive.setPower(-0.5);
            FRDrive.setPower(0.5);
            BLDrive.setPower(-0.5);
            BRDrive.setPower(0.5);
            telemetry.update();
        }
        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);
        colorCam.getColor();

    }
}