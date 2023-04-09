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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems.ChodeLift;
import org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems.IntakeFSM;
import org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems.TeleopInit;
import org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems.LiftZeroFSM;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ChodeTeleop")
public class ChodeTeleop extends OpMode {

    private ChodeDrive drive;
    private Controller controller;
    private Controller controller2;
    private ChodeLift lift;
    private IntakeFSM intakeFSM;
    private LiftZeroFSM liftZero;
    private TeleopInit teleopInit;
    private TouchSensor liftLimit;
    private DistanceSensor intakeSensor;
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
    int intaking = 0;

    public ChodeTeleop() {
    }

    @Override
    public void init() {
        drive = new ChodeDrive(this, hardwareMap);
        controller = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        lift = new ChodeLift();
        intakeFSM = new IntakeFSM();
        teleopInit = new TeleopInit();
        liftZero = new LiftZeroFSM();

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        pivotServo1 = hardwareMap.get(Servo.class, "pivotServo1");
        pivotServo2 = hardwareMap.get(Servo.class, "pivotServo2");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        liftLimit = hardwareMap.get(TouchSensor.class, "liftLimit");
        intakeSensor = hardwareMap.get(DistanceSensor.class, "intakeSensor");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.liftInit(hardwareMap);
        intakeFSM.intakeInit(hardwareMap);
        teleopInit.teleopInit(hardwareMap);
        liftZero.liftZeroInit(hardwareMap, lift);
    }

    @Override
    public void loop() {
        controller.update();
        controller2.update();

        telemetry.addData("LiftLimit", liftLimit.isPressed());
        telemetry.addData("LiftEncdr", lift1.getCurrentPosition());
        telemetry.addData("LiftEncdr2", lift2.getCurrentPosition());
        telemetry.addData("SlowMode?", slowMode);
        telemetry.addData("Lift P", lift.getP());
        telemetry.addData("Lift I", lift.getI());
        telemetry.addData("Lift D", lift.getD());
        telemetry.addData("FF", lift.getFF());
        telemetry.addData("Lift1Power", lift1.getPower());
        telemetry.addData("Lift2Power", lift2.getPower());
        telemetry.addData("PID Setpoint", lift.getSetpoint());
        telemetry.addData("PID Power", lift.getPIDPower());
        telemetry.addData("Encoder Average", lift.getEncoderAverage());
        telemetry.addData("Distance Sensor", intakeSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("leftTriggerOld", leftTrigger);
        telemetry.addData("rightTriggerOld", rightTrigger);
        telemetry.addData("Drive P", drive.getDriveP());
        telemetry.addData("Drive I", drive.getDriveI());
        telemetry.addData("Drive D", drive.getDriveD());
        telemetry.addData("Lift Resting", liftZero.getLiftResting());
        telemetry.addData("Arrived", lift.arrived());
        telemetry.update();

        // Controls
        // Driver 1: Left Trigger to start intaking, release to score, Right Trigger score, Buttons for lift, dpad for fine control
            //bumpers to toggle drive modes
        // Driver 2: Buttons for Lift, Triggers for fine control


        //Drive Controls
        if (controller.rightBumperOnce()) {
            if (slowMode == 0) {
                slowMode = 1;
            } else {
                slowMode = 0;
            }
        }
        if (controller.leftBumperOnce()) {
            drive.FieldCentricToggle();
        }
        if (slowMode == 0) {
            drive.drive(-controller.left_stick_x, -controller.left_stick_y, -controller.right_stick_x);
        } else {
            drive.drive(-controller.left_stick_x/2, -controller.left_stick_y/2, -controller.right_stick_x /2);
        }

        //Scoring
        if (controller.right_trigger > 0.2 && rightTrigger == 0 && controller.left_trigger == 0) {
            liftZero.startLiftZeroFSM();
        }
        liftZero.liftZero();

        //Intake
        if (controller.left_trigger > 0.2 && leftTrigger == 0 && controller.right_trigger == 0) {
            intake1.setPower(.7);
            intake2.setPower(-.7);
            intaking = 1;
        }

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

        // Intake Automation
        if (intakeSensor.getDistance(DistanceUnit.INCH) < 5 && intaking == 1) {
            intake1.setPower(0);
            intake2.setPower(0);
            intaking = 0;
            intakeFSM.resetTimer();
            intakeFSM.startIntakeFSM();
        }
        intakeFSM.intake();

        //Init Automation
        if (controller.left_trigger == 1 && controller.right_trigger == 1) {
            teleopInit.resetTimer();
            teleopInit.startTeleopInit();
        }
        teleopInit.init();

        //Lift PID Toggle and Trigger Toggle
        leftTrigger = controller.left_trigger;
        rightTrigger = controller.right_trigger;
        if (liftZero.getLiftResting() == 0) {
            lift.moveLift();
        }

        //Change Target Position
        if (controller.leftStickButtonOnce()) {
            lift.setLiftPos(1000);
            liftZero.setLiftResting(0);
        } else if (controller.rightStickButtonOnce()) {
            lift.setLiftPos(2000);
            liftZero.setLiftResting(0);
        }
    }
}

//        //PID Tuning
//        // P controlled with bumpers
//        if (controller.dpadLeftOnce()) {
//            lift.changeLiftP(-0.001);
//        //    drive.changeDriveP(-0.001);
//        } else if (controller.dpadRightOnce()) {
//            lift.changeLiftP(0.001);
//        //    drive.changeDriveP(0.001);
//        }
//        // D controlled with A and Y
//        if (controller.AOnce()) {
//            lift.changeLiftD(-0.001);
//        //    drive.changeDriveD(-0.001);
//        } else if (controller.YOnce()) {
//            lift.changeLiftD(0.001);
//        //    drive.changeDriveD(0.001);
//        }
//        // FF controlled with dpad
//        if (controller.dpadDownOnce()) {
//            lift.changeLiftFF(-0.01);
//        } else if (controller.dpadUpOnce()) {
//            lift.changeLiftFF(0.01);
//        }
//        // I controlled with X and B
//        if (controller.XOnce()) {
//            lift.changeLiftI(-0.001);
//        //    drive.changeDriveI(-0.001);
//        } else if (controller.BOnce()) {
//            lift.changeLiftI(0.001);
//        //    drive.changeDriveI(0.001);
//        }
//
//
//        //Manual Lift Control for FF Tuning
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

