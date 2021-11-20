package org.firstinspires.ftc.teamcode.slowBolon;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;


@TeleOp(name = "BOLON teleop")
public class TeleopBolon extends OpMode {

    private DriveBolon drive;
    private Controller controller;
    private Gyro gyro;
    private OdometryBolon o;
    private Servo cat1;
    private Servo cat2;

    private CRServo duckDumpy;
    //private AndroidOrientation orientation;
    private DcMotor lift;
    private DcMotor extend;

    private boolean outok, backok;


    @Override
    public void init() {

        //backok = true; outok = true;
        drive = new DriveBolon(this, hardwareMap);
        drive.setup();
        gyro = new Gyro(hardwareMap, false);
        //orientation = new AndroidOrientation();
        controller = new Controller(gamepad1);
        //drive.togglePOV(true);
        //orientation.startListening();
        gyro.reset();
        cat1 = hardwareMap.get(Servo.class,"cat1");
        cat1.setPosition(0);
        cat2 = hardwareMap.get(Servo.class, "cat2");
        cat2.setPosition(0.8);

        duckDumpy = hardwareMap.get(CRServo.class,"duckDumpy");
        extend = hardwareMap.get(DcMotor.class,"extend");
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift = hardwareMap.get(DcMotor.class,"lift");
        o = new OdometryBolon();
        o.init(hardwareMap.get(DcMotor.class,"frontLeft"));


        o.run();
    }

    @Override
    public void loop() {

        telemetry.addData("milfs", gyro.getAngleDegrees());
        /*telemetry.addData("angle",orientation.getAngle());
        telemetry.addData("angle2", orientation.getAzimuth());
        telemetry.addData("tobias commited arson", orientatigon-/.etMagnitude());
        telemetry.addData("tobias executed for arson", orientation.getPitch());
        telemetry.addData("ben", orientation.getRoll());
        telemetry.addData("available",orientation.isAvailable());*/

        telemetry.addData("milfs",gyro.getAngleDegrees());
        telemetry.addData("distance",o.distance);
        telemetry.addData("extender-check", extend.getCurrentPosition());
        //telemetry.addData("OUTWARDS",controller.dpadRight() && outok);
        //telemetry.addData("INWARDS",controller.dpadLeft() && backok);

        controller.update();
        drive.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);
        drive.togglePOV(controller.leftStickButtonOnce());
        if(controller.dpadDownOnce()) {o.resetdistance();}

        //outok = (extend.getCurrentPosition()<1680);
        //backok = (extend.getCurrentPosition()>-2);

        if(controller.leftBumperOnce()) {cat1.setPosition(0.4); cat2.setPosition(0.5);}
        if(controller.rightBumperOnce()){cat1.setPosition(0); cat2.setPosition(0.8);}

        if(controller.rightBumper()) {lift.setDirection(DcMotor.Direction.FORWARD);lift.setPower(0.4);}
        else if(controller.leftBumper()) {lift.setDirection(DcMotor.Direction.REVERSE);lift.setPower(0.4);}
        else{lift.setPower(0);}

        if(controller.rightBumper()) {extend.setDirection(DcMotor.Direction.FORWARD);extend.setPower(0.4);}
        else if(controller.leftBumper()) {extend.setDirection(DcMotor.Direction.REVERSE);extend.setPower(0.4);}
        else{extend.setPower(0);}

        //if(controller.dpadRight()/* && (outok==true)*/) {extend.setDirection(DcMotor.Direction.FORWARD);extend.setPower(1);}
        //else if(controller.dpadLeft()/* && (backok==true)*/) {extend.setDirection(DcMotor.Direction.REVERSE);extend.setPower(1);}
        //else{extend.setPower(0);}

        if(controller.B()) {duckDumpy.setPower(1);} else{duckDumpy.setPower(0);}
        telemetry.update();
        if(controller.XOnce()) {telemetry.speak("que deporte te gusta me gusta el beisbol");}
    }

}
