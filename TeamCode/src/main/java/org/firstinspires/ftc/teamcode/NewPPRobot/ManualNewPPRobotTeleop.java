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

@TeleOp(name = "New Powerplay Robot TeleOp")
public class ManualNewPPRobotTeleop extends OpMode {

    private NewPPDrive drive;
    private Controller controller;
    private Controller controller2;
    private ServoEx clawServo;
    private ServoEx pivotServo;
    private ServoEx wristServo1;
    private ServoEx wristServo2;
    private DcMotor liftMotor;
    private DcMotor liftMotor2;
    private DcMotor spinMotor;
    private DcMotor spinMotor2;
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

        clawServo = hardwareMap.get(SimpleServo.class, "clawServo");
        pivotServo = hardwareMap.get(SimpleServo.class, "pivotServo");
        wristServo1 = hardwareMap.get(SimpleServo.class, "wristServo1");
        wristServo2 = hardwareMap.get(SimpleServo.class, "wristServo2");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotor.class, "LiftMotor2");
    }

    @Override
    public void loop() {
        controller.update();
        controller2.update();

        telemetry.addData("LiftEncdr", liftMotor.getCurrentPosition());
        telemetry.addData("LiftEncdr2", liftMotor2.getCurrentPosition());
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
            liftMotor.setPower(1);
            liftMotor2.setPower(-1);
        } else if (controller.right_trigger == 1) {
            liftMotor.setPower(-1);
            liftMotor2.setPower(1);
        } else {
            liftMotor.setPower(0);
            liftMotor2.setPower(0);
        }

        //Claw Controls
        if (controller.AOnce()) {
            clawServo.setPosition(135);
        } else if (controller.BOnce()) {
            clawServo.setPosition(85);
        }

        //Intake Controls
        if (controller.leftStickButtonOnce()) {
            spinMotor.setPower(1);
            spinMotor2.setPower(-1);
        } else if (controller.rightStickButtonOnce()) {
            spinMotor.setPower(0);
            spinMotor2.setPower(0);
        }

        //Wrist Pivot Controls
        if (controller.leftBumperOnce()) {
            wristServo1.setPosition(360);
            wristServo2.setPosition(0);
        } else if (controller.rightBumperOnce()) {
            wristServo2.setPosition(360);
            wristServo1.setPosition(0);
        }

        //Wrist Twist Controls
        if (controller.XOnce()) {
            pivotServo.setPosition(21);
        } else if (controller.YOnce()) {
            pivotServo.setPosition(256);
        }
    }
}

