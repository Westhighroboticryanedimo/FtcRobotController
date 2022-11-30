package org.firstinspires.ftc.teamcode.MF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "MF TeleOp")
public class MFTeleop extends OpMode {

    private MFDrive drive;
    private Controller controller;
    private Servo clawServo;
    private DcMotor liftMotor;
    private DcMotor liftMotor2;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    public MFTeleop() {
    }


    @Override
    public void init() {
        drive = new MFDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("LiftEncdr", liftMotor.getCurrentPosition());
        telemetry.addData("DriveEncdrFL", frontLeft.getCurrentPosition());
        telemetry.addData("DriveEncdrFR", frontRight.getCurrentPosition());
        telemetry.addData("DriveEncdrBL", backLeft.getCurrentPosition());
        telemetry.addData("DriveEncdrBR", backRight.getCurrentPosition());
        telemetry.update();
        controller.update();
        drive.drive(-controller.left_stick_x, -controller.left_stick_y, -controller.right_stick_x);
//        telemetry.addData("gyro", gyrog)

        controller.update();
        if (controller.dpadUp()) {
            if (liftMotor.getCurrentPosition() > 3000) {
                liftMotor.setPower(0.5);
                liftMotor2.setPower(-0.5);
            } else {
                liftMotor.setPower(1);
                liftMotor2.setPower(-1);
            }
        } else if (controller.dpadDown()) {
            if (liftMotor.getCurrentPosition() < 300) {
                liftMotor.setPower(0);
                liftMotor2.setPower(0);
            } else if (liftMotor.getCurrentPosition() < 1200) {
                liftMotor.setPower(-0.5);
                liftMotor2.setPower(0.5);
            } else {
                liftMotor.setPower(-1);
                liftMotor2.setPower(1);
            }
        } else {
            liftMotor.setPower(0);
            liftMotor2.setPower(0);
        }
//        if (controller.dpadDown() && liftMotor.getCurrentPosition() < 300) {
//            liftMotor.setPower(0);
//            liftMotor2.setPower(0);
//        }
//        if (controller.dpadUp()) {
//            liftMotor.setPower(0.5);
//            liftMotor2.setPower(0.5);
//        } else if (controller.dpadDown()) {
//            liftMotor.setPower(-0.5);
//            liftMotor2.setPower(-0.5);
//        } else {
//            liftMotor.setPower(0);
//            liftMotor2.setPower(0);
//        }

//        if (controller.BOnce()) {
//            clawServo.setPosition(0.65);
//        }
//        if (controller.AOnce()) {
//            clawServo.setPosition(0);
//        }
    }
}

