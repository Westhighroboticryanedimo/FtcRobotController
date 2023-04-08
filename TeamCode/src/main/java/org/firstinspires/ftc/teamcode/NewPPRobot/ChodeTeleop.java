// TODO: Add PID tuning with controller (methods for changing variables are done already)
// TODO: Update preset values, add presets for stack
// TODO: Add slow descent + encoder reset on limit switch
// TODO: Set up all FSMs
// TODO: Tune drive PID
package org.firstinspires.ftc.teamcode.NewPPRobot;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems.ChodeLift;

@TeleOp(name = "ChodeTeleop")
public class ChodeTeleop extends OpMode {

    private ChodeDrive drive;
    private Controller controller;
    private Controller controller2;
    private ChodeLift lift;
    private TouchSensor liftLimit;
    private Servo clawServo;
    private Servo wristServo;
    private Servo pivotServo1;
    private Servo pivotServo2;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    double leftTrigger = 0;
    double rightTrigger = 0;
    int slowMode = 0;
    int liftResting = 1;
    int intaking = 0;

    public ChodeTeleop() {
    }

    @Override
    public void init() {
        drive = new ChodeDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        lift = new ChodeLift();

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        pivotServo1 = hardwareMap.get(Servo.class, "pivotServo1");
        pivotServo2 = hardwareMap.get(Servo.class, "pivotServo2");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        liftLimit = hardwareMap.get(TouchSensor.class, "liftLimit");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        telemetry.addData("LiftEncdr", lift1.getCurrentPosition());
        telemetry.addData("LiftEncdr2", lift2.getCurrentPosition());
        telemetry.addData("SlowMode?", slowMode);
        telemetry.addData("Resting?", liftResting);
        telemetry.addData("P", lift.getP());
        telemetry.addData("I", lift.getI());
        telemetry.addData("D", lift.getD());
        telemetry.addData("FF", lift.getFF());
        telemetry.addData("Lift1Power", lift1.getPower());
        telemetry.addData("Lift2Power", lift2.getPower());
        telemetry.addData("PID Setpoint", lift.getSetpoint());
        telemetry.addData("PID Power", lift.getPIDPower());
        telemetry.update();

        // Controls
        // Driver 1: Left Trigger to start intaking, release to score, Right Trigger score, Buttons for lift, dpad for fine control
            //bumpers to toggle drive modes
        // Driver 2: Buttons for Lift, Triggers for fine control


        //Drive Controls
//        if (controller.rightBumperOnce()) {
//            if (slowMode == 0) {
//                slowMode = 1;
//            } else {
//                slowMode = 0;
//            }
//        }
//        if (controller.leftBumperOnce()) {
//            drive.FieldCentricToggle();
//        }
//        if (slowMode == 0) {
//            drive.drive(-controller.left_stick_x, -controller.left_stick_y, -controller.right_stick_x);
//        } else {
//            drive.drive(-controller.left_stick_x/2, -controller.left_stick_y/2, -controller.right_stick_x /2);
//        }

        //Scoring
//        if (controller.right_trigger == 1 && rightTrigger == 0) {
//            // TODO: FSM sequence to dunk cone + tilt claw, open claw, pivot back, lower lift
//        }

        //Intake
//        if (controller.left_trigger == 1 && leftTrigger == 0) {
//            spinMotor.setPower(.5);
//            spinMotor2.setPower(-.5);
//            intaking = 1;
//        }

        //Lift
//        if (controller.XOnce() || controller2.XOnce()) {
//            lift.setLiftPos(100);
//            liftResting = 0;
//        } else if (controller.YOnce() || controller2.YOnce()) {
//            lift.setLiftPos(200);
//            liftResting = 0;
//        } else if (controller.BOnce() || controller2.BOnce()) {
//            lift.setLiftPos(300);
//            liftResting = 0;
//        } else if (controller.AOnce() || controller2.AOnce()) {
//            // TODO: FSM sequence to lower the lift to ~ 1" above limit, then slowly lower until limit is pressed, then reset encoders
//            // TODO: Change "resting" variable to 1 toggle PID off
//        }

        //Intake on Color Sensor Activation
//        if (intaking == 1) {
//            // TODO: Check color sensor and activate intake FSM when triggered
//        }

        //PID Tuning
        // P controlled with bumpers
        if (controller.leftBumperOnce()) {
            lift.changeLiftP(-0.01);
        } else if (controller.rightBumperOnce()) {
            lift.changeLiftP(0.01);
        }
        // D controlled with A and Y
        if (controller.AOnce()) {
            lift.changeLiftD(-0.01);
        } else if (controller.YOnce()) {
            lift.changeLiftD(0.01);
        }
        // FF controlled with dpad
        if (controller.dpadDownOnce()) {
            lift.changeLiftFF(-0.01);
        } else if (controller.dpadUpOnce()) {
            lift.changeLiftFF(0.01);
        }
        // I controlled with X and B
        if (controller.XOnce()) {
            lift.changeLiftI(-0.01);
        } else if (controller.BOnce()) {
            lift.changeLiftI(0.01);
        }

        //Change Target Position
        if (controller.leftStickButtonOnce()) {
            lift.setLiftPos(500);
            liftResting = 0;
        } else if (controller.rightStickButtonOnce()) {
            lift.setLiftPos(1000);
            liftResting = 0;
        }

        //Manual Lift Control for FF Tuning
//        if (controller.left_trigger == 1) {
//            lift1.setPower(-.5);
//            lift2.setPower(.5);
//            liftResting = 1;
//        } else if (controller.right_trigger == 1) {
//            lift1.setPower(1);
//            lift2.setPower(-1);
//            liftResting = 1;
//        } else {
//            liftResting = 0;
//        }

        //Lift PID Toggle
        leftTrigger = controller.left_trigger;
        rightTrigger = controller.right_trigger;
        if (liftResting == 0) {
            lift.moveLift();
        }
    }
}

