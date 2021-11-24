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
    private boolean slowbol;
    private double speed;

    private boolean handopen;

    @Override
    public void init() {
        speed=1; handopen = true;
        slowbol = false;
        backok = true; outok = true;
        drive = new DriveBolon(this, hardwareMap);
        drive.setup();
        gyro = new Gyro(hardwareMap,false);
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
        //extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        //telemetry.addData("milfs",gyro.getAngleDegrees());
        //telemetry.addData("distance",o.distance);
        telemetry.addData("extender-check", extend.getCurrentPosition());
        telemetry.addData("OUTWARDS",controller.right_trigger>0.2&&outok);
        telemetry.addData("INWARDS",controller.left_trigger>0.2&&backok);
        telemetry.addData("version","30");

        controller.update();
        drive.drive(controller.left_stick_x*speed, controller.left_stick_y*speed, controller.right_stick_x);
        //drive.togglePOV(controller.leftStickButtonOnce());
        if(controller.dpadDownOnce()) {o.resetdistance();}

        outok = (extend.getCurrentPosition()<1600);//real number around 1690
        backok = (extend.getCurrentPosition()>-2);

        if(controller.leftStickButtonOnce()) {slowbol = !slowbol;}
        if(slowbol) {speed=0.4;} else {speed=1;}

        if(controller.rightBumperOnce()) {handopen = !handopen;}

        if(!handopen) {cat1.setPosition(0.4); cat2.setPosition(0.5);}
        else {cat1.setPosition(0); cat2.setPosition(0.8);}

        if(controller.dpadUp()) {lift.setDirection(DcMotor.Direction.FORWARD);lift.setPower(0.37*speed);}
        else if(controller.dpadDown()) {lift.setDirection(DcMotor.Direction.REVERSE);lift.setPower(0.27*speed);}
        else{lift.setPower(0);}

        if(controller.right_trigger>0.2&&outok) {extend.setPower(0.4*speed);}
        else if(controller.left_trigger>0.2&&backok) {extend.setPower(-0.4*speed);}
        else{extend.setPower(0);}

        if(controller.B()) {duckDumpy.setDirection(DcMotorSimple.Direction.FORWARD);duckDumpy.setPower(1);}
        else if(controller.Y()) {duckDumpy.setDirection(DcMotorSimple.Direction.REVERSE);duckDumpy.setPower(1);}
        else{duckDumpy.setPower(0);}
        telemetry.update();
        if(controller.XOnce()) {telemetry.speak("que deporte te gusta me gusta el beisbol");}
    }

}
