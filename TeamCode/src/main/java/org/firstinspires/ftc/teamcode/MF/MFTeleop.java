package org.firstinspires.ftc.teamcode.MF;

import android.util.Log;

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

    @Override
    public void init() {
        drive = new MFDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Log.d("MFers_LiftEncdr",
                String.format("Initial Encoder pos: %d",
                        liftMotor.getCurrentPosition()
                ) );
    }

    @Override
    public void loop() {
        controller.update();
        drive.drive(-controller.left_stick_x, -controller.left_stick_y, -controller.right_stick_x);
//        telemetry.addData("gyro", gyrog)
        controller.update();
        if (controller.dpadDown() && liftMotor.getCurrentPosition() < 0) {
            liftMotor.setPower(0);
            Log.d("MFers_LiftEncdr",
                    String.format("dpadDown and below 0 pos: %d",
                            liftMotor.getCurrentPosition()
                    ) );
        } else if (controller.dpadUp()){
            liftMotor.setPower(-0.5);
            Log.d("MFers_LiftEncdr",
                    String.format("dpadUp pos: %d",
                            liftMotor.getCurrentPosition()
                    ) );
        } else if (controller.dpadDown()){
            liftMotor.setPower(0.5);
            Log.d("MFers_LiftEncdr",
                    String.format("dpadDown pos: %d",
                            liftMotor.getCurrentPosition()
                    ) );
        } else {
            liftMotor.setPower(0);
        }
        if (controller.BOnce()){
            clawServo.setPosition(0.65);
        }
        if (controller.AOnce()){
            clawServo.setPosition(0);
        }
    }
}
