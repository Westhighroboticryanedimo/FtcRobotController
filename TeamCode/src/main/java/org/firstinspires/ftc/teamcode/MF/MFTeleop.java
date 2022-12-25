package org.firstinspires.ftc.teamcode.MF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.MF.subsystems.MFDrive;
import org.firstinspires.ftc.teamcode.MF.subsystems.Lift;

@TeleOp(name = "MF TeleOp")
public class MFTeleop extends OpMode {

    private MFDrive drive;
    private Controller controller;
//    private Controller controller2;
    private TouchSensor liftLimit;
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
        Lift lift = new Lift();
//        controller2 = new Controller(gamepad2);
        liftLimit = hardwareMap.get(TouchSensor.class, "liftLimit");
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
        int limit;
        if (liftLimit.isPressed()) {
            limit = 1;
        } else {
            limit = 0;
        }
        telemetry.addData("LiftLimit", limit);
        telemetry.addData("LiftEncdr", liftMotor.getCurrentPosition());
        telemetry.addData("DriveEncdrFL", frontLeft.getCurrentPosition());
        telemetry.addData("DriveEncdrFR", frontRight.getCurrentPosition());
        telemetry.addData("DriveEncdrBL", backLeft.getCurrentPosition());
        telemetry.addData("DriveEncdrBR", backRight.getCurrentPosition());
        telemetry.update();
        controller.update();
        if (controller.rightStickButton()) {
            drive.drive(-controller.left_stick_x*1/2, -controller.left_stick_y*1/2, -controller.right_stick_x);
        } else {
            drive.drive(-controller.left_stick_x, -controller.left_stick_y, -controller.right_stick_x);
        }
//        drive.drive(-controller.left_stick_x, -controller.left_stick_y, -controller.right_stick_x);

//        telemetry.addData("gyro", gyrog)

        controller.update();
//        controller2.update();
        if (controller.dpadUp()) {
            if (liftMotor.getCurrentPosition() < -3862) {
                liftMotor.setPower(0);
                liftMotor2.setPower(0);
            } else {
                liftMotor.setPower(-1);
                liftMotor2.setPower(1);
            }

        } else if (controller.dpadDown()) {
            if (liftLimit.isPressed()) {
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor.setPower(0);
                liftMotor2.setPower(0);
            } else {
                liftMotor.setPower(0.5);
                liftMotor2.setPower(-0.5);
            }
        } else {
            liftMotor.setPower(0);
            liftMotor2.setPower(0);
        }

        if (controller.AOnce()) {
            clawServo.setPosition(0);
        } else if (controller.BOnce()) {
            clawServo.setPosition(0.65);
        }
    }
}

