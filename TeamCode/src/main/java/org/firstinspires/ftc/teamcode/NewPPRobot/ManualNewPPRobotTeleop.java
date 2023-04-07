// TODO: Add PID tuning with controller (methods for changing variables are done already)
// TODO: Update preset values, add presets for stack
// TODO: Add slow descent + encoder reset on limit switch
// TODO: Set up all FSMs
// TODO: Tune servo positions for FSMs
// TODO: Tune drive PID
package org.firstinspires.ftc.teamcode.NewPPRobot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "ManualChodeTeleop")
public class ManualNewPPRobotTeleop extends OpMode {

    private NewPPDrive drive;
    private Controller controller;
    private Controller controller2;
    private ServoEx clawServo;
    private ServoEx wristServo;
    private ServoEx pivotServo1;
    private ServoEx pivotServo2;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    int slowMode = 0;

    public ManualNewPPRobotTeleop() {
    }

    @Override
    public void init() {
        drive = new NewPPDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 360);
        wristServo = new SimpleServo(hardwareMap, "wristServo", 0, 360);
        pivotServo1 = new SimpleServo(hardwareMap, "pivotServo1", 0, 360);
        pivotServo2 = new SimpleServo(hardwareMap, "pivotServo2", 0, 360);
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
    }

    @Override
    public void loop() {
        controller.update();
        controller2.update();

        telemetry.addData("LiftEncdr", lift1.getCurrentPosition());
        telemetry.addData("LiftEncdr2", lift2.getCurrentPosition());
        telemetry.addData("SlowMode?", slowMode);
        telemetry.update();

        // Controls
        // Driver 1: Left Trigger hold to intake, release to score, Right Trigger score
        // Driver 2: Buttons for Lift, Triggers for fine control

        //Drive Controls
        if (controller.leftBumperOnce()) {
            drive.FieldCentricToggle();
        }
        if (slowMode == 0) {
            drive.drive(-controller.left_stick_x, -controller.left_stick_y, -controller.right_stick_x);
        } else {
            drive.drive(-controller.left_stick_x/2, -controller.left_stick_y/2, -controller.right_stick_x /2);
        }

        //Lift Controls
        if (controller.left_trigger == 1) {
            lift1.setPower(1);
            lift2.setPower(-1);
        } else if (controller.right_trigger == 1) {
            lift1.setPower(-1);
            lift2.setPower(1);
        } else {
            lift1.setPower(0);
            lift2.setPower(0);
        }

        //Claw Controls
        if (controller.AOnce()) {
            clawServo.setPosition(135);
        } else if (controller.BOnce()) {
            clawServo.setPosition(85);
        }

        //Intake Controls
        if (controller.leftStickButtonOnce()) {
            intake1.setPower(.5);
            intake2.setPower(-.5);
        } else if (controller.rightStickButtonOnce()) {
            intake1.setPower(0);
            intake2.setPower(0);
        }

        //Wrist Pivot Controls
        if (controller.leftBumperOnce()) {
            pivotServo1.setPosition(360);
            pivotServo2.setPosition(0);
        } else if (controller.rightBumperOnce()) {
            pivotServo2.setPosition(360);
            pivotServo1.setPosition(0);
        }

        //Wrist Twist Controls
        if (controller.XOnce()) {
            wristServo.setPosition(21);
        } else if (controller.YOnce()) {
            wristServo.setPosition(256);
        }
    }
}

