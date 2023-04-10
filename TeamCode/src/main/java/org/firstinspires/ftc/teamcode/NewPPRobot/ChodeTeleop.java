// TODO: Add PID tuning with controller (methods for changing variables are done already)
// TODO: Update preset values, add presets for stack
// TODO: Add slow descent + encoder reset on limit switch
// TODO: Set up all FSMs
// TODO: Tune drive PID
package org.firstinspires.ftc.teamcode.NewPPRobot;

import static java.lang.Math.abs;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems.ChodeLift;
import org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems.IntakeFSM;
import org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems.TeleopInit;
import org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems.LiftZeroFSM;
import org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems.OuttakeFSM;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.NewPPRobot.ChodeDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

@TeleOp(name = "ChodeTeleop")
public class ChodeTeleop extends OpMode {

    private ChodeDrive drive;
    private Controller controller;
    private Controller controller2;
    private ChodeLift lift;
    private IntakeFSM intakeFSM;
    private OuttakeFSM outtakeFSM;
    private LiftZeroFSM liftZero;
    private TeleopInit teleopInit;
    private TouchSensor liftLimit;
    private DistanceSensor intakeSensor;
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
        outtakeFSM = new OuttakeFSM();
        teleopInit = new TeleopInit();
        liftZero = new LiftZeroFSM();

        clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 360);
        wristServo = new SimpleServo(hardwareMap, "wristServo", 0, 360);
        pivotServo1 = new SimpleServo(hardwareMap, "pivotServo1", 0, 360);
        pivotServo2 = new SimpleServo(hardwareMap, "pivotServo2", 0, 360);
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
        outtakeFSM.outtakeInit(hardwareMap);
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
//        if (controller.leftBumperOnce()) {
//            drive.FieldCentricToggle();
//        }
        if (slowMode == 0) {
            drive.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
        } else {
            drive.drive(controller.left_stick_x/2, controller.left_stick_y/2, controller.right_stick_x /2);
        }

        //Scoring
        if (controller.right_trigger > 0.5 && rightTrigger == 0 && controller.left_trigger == 0) {
            liftZero.startLiftZeroFSM();
            outtakeFSM.startOuttakeFSM();
            liftZero.setLiftResting(0);
            slowMode = 0;
        }
        liftZero.liftZero();
        outtakeFSM.outtake();

        //Intake
        if (controller.left_trigger > 0.5 && leftTrigger == 0 && controller.right_trigger == 0) {
            intake1.setPower(.7);
            intake2.setPower(-.7);
            intaking = 1;
        }

        // Intake Automation
        if (intakeSensor.getDistance(DistanceUnit.INCH) < 5 && intaking == 1) {
            intake1.setPower(0);
            intake2.setPower(0);
            intaking = 0;
            intakeFSM.resetTimer();
            intakeFSM.startIntakeFSM();
        }
        intakeFSM.intake();

        //Reset Automation
        if (controller.left_trigger == 1 && controller.right_trigger == 1) {
            teleopInit.resetTimer();
            teleopInit.startTeleopInit();
        }
        teleopInit.init();

        //Raising Lift
        if (controller.XOnce()) {
            lift.setLiftPos(700);
            liftZero.setLiftResting(0);
        } else if (controller.YOnce()) {
            lift.setLiftPos(1550);
            liftZero.setLiftResting(0);
        } else if (controller.BOnce()) {
            lift.setLiftPos(2350);
            liftZero.setLiftResting(0);
        }
        if (lift.getSetpoint() == 700 || lift.getSetpoint() == 1550 || lift.getSetpoint() == 2350) {
            pivotServo1.turnToAngle(310);
            pivotServo2.turnToAngle(50);
        }

        //Manual Lift Control
        if (controller.dpadUpOnce()) {
            lift.setLiftPos(lift.getSetpoint() + 50);
        }

        //Intake Reverse
        if (controller.startOnce() && intake1.getPower() == 0) {
            intake1.setPower(-0.2);
            intake2.setPower(0.2);
        } else if (controller.backOnce() && intake1.getPower() == 0) {
            intake1.setPower(-0.7);
            intake2.setPower(0.7);
        } else if ((controller.startOnce() || controller.backOnce()) && intake1.getPower() < 1) {
            intake1.setPower(0);
            intake2.setPower(0);
            teleopInit.resetTimer();
            teleopInit.startTeleopInit();
        }

        //Lift PID Toggle and Trigger Toggle
        if (controller.left_trigger < 0.1) {
            leftTrigger = 0;
        }
        if (controller.right_trigger < 0.1) {
            rightTrigger = 0;
        }
        if (liftZero.getLiftResting() == 0) {
            lift.moveLift();
        }
    }
}

//        //PID Tuning
//        // P controlled with bumpers
//        if (controller2.dpadLeftOnce()) {
//            lift.changeLiftP(-0.001);
//        //    drive.changeDriveP(-0.001);
//        } else if (controller2.dpadRightOnce()) {
//            lift.changeLiftP(0.001);
//        //    drive.changeDriveP(0.001);
//        }
//        // D controlled with A and Y
//        if (controller2.AOnce()) {
//            lift.changeLiftD(-0.001);
//        //    drive.changeDriveD(-0.001);
//        } else if (controller2.YOnce()) {
//            lift.changeLiftD(0.001);
//        //    drive.changeDriveD(0.001);
//        }
//        // FF controlled with dpad
//        if (controller2.dpadDownOnce()) {
//            lift.changeLiftFF(-0.01);
//        } else if (controller2.dpadUpOnce()) {
//            lift.changeLiftFF(0.01);
//        }
//        // I controlled with X and B
//        if (controller2.XOnce()) {
//            lift.changeLiftI(-0.001);
//        //    drive.changeDriveI(-0.001);
//        } else if (controller2.BOnce()) {
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

