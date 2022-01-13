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
    private CRServo newgrab1, newgrab2;

    private CRServo duckDumpy;
    //private AndroidOrientation orientation;
    private DcMotor lift;
    private DcMotor extend;

    private boolean outok, backok;
    private int autoextend; // 0 = nothing 1 = in 2 = out
    private boolean slowbol;
    private double speed;

    private boolean handopen;

    @Override
    public void init() {
        autoextend = 0;
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
        cat1.setDirection(Servo.Direction.FORWARD);
        cat2 = hardwareMap.get(Servo.class, "cat2");
        cat2.setPosition(0.8);
        cat2.setDirection(Servo.Direction.FORWARD);

        newgrab1 = hardwareMap.get(CRServo.class, "grab1");
        newgrab2 = hardwareMap.get(CRServo.class, "grab2");
        newgrab1.setDirection(CRServo.Direction.FORWARD); newgrab2.setDirection(CRServo.Direction.REVERSE);

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
        telemetry.addData("bee boo",autoextend);
        telemetry.addData("version","31");

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

        if(controller.dpadUp()) {lift.setDirection(DcMotor.Direction.FORWARD);lift.setPower(0.37);}
        else if(controller.dpadDown()) {lift.setDirection(DcMotor.Direction.REVERSE);lift.setPower(0.27);}
        else{lift.setPower(0);}

        if(controller.leftBumperOnce()) {
            if(autoextend == 0) {
                // if in middle, go back, if out, go back, it in, go out
                if(outok && backok) {autoextend = 1;}
                if(!outok) {autoextend = 1;}
                if(!backok) {autoextend = 2;}
            }
            //if(autoextend != 0) {autoextend = 0;}
        }

        if(autoextend == 2 && outok) {extend.setPower(1);}
        else if(autoextend == 2 && !outok) {autoextend = 0;extend.setPower(0);}
        else if(autoextend == 1 && (extend.getCurrentPosition()>7)) {extend.setPower(-0.7);}
        else if(autoextend == 1 && !backok) {autoextend = 0;extend.setPower(0);}
        else if(controller.right_trigger>0.2&&outok) {extend.setPower(0.4*speed);}
        else if(controller.left_trigger>0.2&&backok) {extend.setPower(-0.4*speed);}
        else{extend.setPower(0);}

        if(controller.B()) {duckDumpy.setDirection(DcMotorSimple.Direction.FORWARD);duckDumpy.setPower(1);
        newgrab1.setDirection(CRServo.Direction.FORWARD);newgrab2.setDirection(CRServo.Direction.REVERSE);
        newgrab1.setPower(1);
        newgrab2.setPower(1);}
        else if(controller.Y()) {duckDumpy.setDirection(DcMotorSimple.Direction.REVERSE);duckDumpy.setPower(1);

        newgrab1.setDirection(CRServo.Direction.FORWARD);newgrab2.setDirection(CRServo.Direction.REVERSE);
        newgrab2.setPower(1);newgrab1.setPower(1);}
        else{duckDumpy.setPower(0);newgrab1.setPower(0);newgrab2.setPower(0);}
        telemetry.update();
        if(controller.XOnce()) {telemetry.speak("que deporte te gusta me gusta el beisbol");}
    }

}

// removed