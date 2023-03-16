// TODO: Add PID tuning with controller (methods for changing variables are done already)
// TODO: Update preset values, add presets for stack, change intake preset value in Subsystem.Lift for special ff case
// TODO: Add slow descent + encoder reset on limit switch
// TODO: Set up drive code with toggle for Field Centric -> Robot Centric -> Robot Centric Slow Mode
// TODO: Set up all FSMs
// TODO: Tune servo positions for FSMs
package org.firstinspires.ftc.teamcode.NewPPRobot;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.NewPPRobot.NewPPDrive;
import org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems.Lift;

@TeleOp(name = "New Powerplay Robot TeleOp")
public class NewPPTeleop extends OpMode {

    private NewPPDrive drive;
    private Controller controller;
    private Controller controller2;
    private Lift lift;
    private TouchSensor liftLimit;
    private Servo clawServo;
    private Servo pivotServo;
    private Servo wristServo1;
    private Servo wristServo2;
    private DcMotor liftMotor;
    private DcMotor liftMotor2;
    private DcMotor spinMotor;
    private DcMotor spinMotor2;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    double leftTrigger = 0;
    double rightTrigger = 0;

    public NewPPTeleop() {
    }

    private class liftEncdr {
        private int getEncoders() {
            return((abs(liftMotor.getCurrentPosition())+abs(liftMotor2.getCurrentPosition()))/2);
        }
    }
    private NewPPTeleop.liftEncdr liftencdr;

    @Override
    public void init() {
        drive = new NewPPDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        lift = new Lift();

        liftLimit = hardwareMap.get(TouchSensor.class, "liftLimit");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        wristServo1 = hardwareMap.get(Servo.class, "wristServo1");
        wristServo2 = hardwareMap.get(Servo.class, "wristServo2");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.liftInit(hardwareMap);

    }

    @Override
    public void loop() {
        controller.update();
        controller2.update();

        int limit;
        if (liftLimit.isPressed()) {
            limit = 1;
        } else {
            limit = 0;
        }

        telemetry.addData("LiftLimit", limit);
        telemetry.addData("LiftEncdr", liftMotor.getCurrentPosition());
        telemetry.addData("LiftEncdr2", liftMotor2.getCurrentPosition());
        telemetry.update();

        // Controls
        // Driver 1: Left Trigger hold to intake, release to score, Right Trigger score
        // Driver 2: Dpad Lift, Triggers for fine control
        if (controller.rightStickButton()) {
            drive.drive(-controller.left_stick_x * 1 / 4, -controller.left_stick_y * 1 / 4, -controller.right_stick_x * 1 / 3);
        } else {
            drive.drive(-controller.left_stick_x * 2 / 3, -controller.left_stick_y * 2 / 3, -controller.right_stick_x * 2 / 3);
        }

        // controller2 lift presets
        // controller2 manual lift controls

        if (controller.right_trigger == 1 && rightTrigger == 0) {
            //FSM sequence to dunk cone + tilt claw, open claw, pivot back, lower lift
        }

        if (controller.left_trigger == 1) {
            spinMotor.setPower(1);
            spinMotor2.setPower(-1);
        }
        if (controller.left_trigger == 0 && leftTrigger == 1) {
            spinMotor.setPower(0);
            spinMotor.setPower(0);
            //FSM sequence to close claw, pivot wrist up
        }

        if (controller.XOnce() || controller2.XOnce()) {
            lift.setLiftPos(100);
        } else if (controller.YOnce() || controller2.YOnce()) {
            lift.setLiftPos(200);
        } else if (controller.BOnce() || controller2.BOnce()) {
            lift.setLiftPos(300);
        } else if (controller.AOnce() || controller2.AOnce()) {
            //FSM sequence to lower the lift to ~ 1" above limit, then slowly lower until limit is pressed, then reset encoders
        }

        leftTrigger = controller.left_trigger;
        rightTrigger = controller.right_trigger;
        lift.moveLift();
    }
}

