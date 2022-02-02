package org.firstinspires.ftc.teamcode.slowBolon;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;


@TeleOp(name = "smol bolon teleop")
public class TeleopBolon extends OpMode {

    private DriveBolon drive;
    private Controller controller;
    //private Controller controller2;
    private Gyro gyro;
    private CRServo newgrab1, newgrab2;
    private DcMotor duckDumpy;
    private DcMotor lift;

    DcMotor fl, fr, br, bl;

    private boolean slowbol;
    private double speed;
    private double smoothing;
    private static double SMOOTH_RATE;
    private double stick_x, stick_y;
    private double armposition;
    private int armupposition = 3000, armdownposition = 0;
    private int armgoalposition;

    private boolean handopen;

    private Bolodometry bd;

    @Override
    public void init() {
        smoothing = 0;
        SMOOTH_RATE = 0.1;
        stick_x = 0; stick_y = 0;
        armgoalposition = -1;
        armupposition = 3000; armdownposition = 0;
        speed=1; handopen = true;
        slowbol = false;
        drive = new DriveBolon(this, hardwareMap);
        drive.setup();
        gyro = new Gyro(hardwareMap,false);
        controller = new Controller(gamepad1);
        //controller2 = new Controller(gamepad2);
        gyro.reset();
        newgrab1 = hardwareMap.get(CRServo.class, "grab1");
        newgrab2 = hardwareMap.get(CRServo.class, "grab2");
        bd = new Bolodometry();
        bd.init();

        duckDumpy = hardwareMap.get(DcMotor.class,"duckDumpy");

        lift = hardwareMap.get(DcMotor.class,"lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl = hardwareMap.get(DcMotor.class,"frontLeft");
        fr = hardwareMap.get(DcMotor.class,"frontRight");
        bl = hardwareMap.get(DcMotor.class,"backLeft");
        br = hardwareMap.get(DcMotor.class,"backRight");
    }

    @Override
    public void loop() {
        bd.updateposition(fr.getCurrentPosition(),fl.getCurrentPosition(),br.getCurrentPosition(),bl.getCurrentPosition(),gyro.getAngleDegrees());
        armposition = lift.getCurrentPosition();

        if(armgoalposition != -1) {
            if(armposition > armgoalposition) {lift.setPower(-0.4);}
            else if(armposition < armgoalposition) {lift.setPower(0.4);}

            if(armposition >= armgoalposition && armgoalposition == armupposition) {armgoalposition = -1;}
            if(armposition <= armgoalposition && armgoalposition == armdownposition) {armgoalposition = -1;}
        }
        if(controller.left_trigger>0.2) {
            //telemetry.addData("PRESSING","");
            if(armgoalposition != armupposition && armgoalposition != armdownposition) {
                telemetry.addData("pressing","");
                armgoalposition = 3000;
                telemetry.addData("i am a bad programer","");
            }
            else if(armgoalposition == armupposition) {armgoalposition = 0;}
            else if(armgoalposition == armdownposition) {armgoalposition = -1;}
        }

        telemetry.addData("milfs", gyro.getAngleDegrees());
        telemetry.addData("Bolodometry y: ",bd.y);
        telemetry.addData("smoothing: ", smoothing);
        telemetry.addData("l: ", "stick_x " + stick_x + " stick_y" + stick_y);
        telemetry.addData("FL",fl.getCurrentPosition());
        telemetry.addData("FR",fr.getCurrentPosition());
        telemetry.addData("BR",br.getCurrentPosition());
        telemetry.addData("BL",bl.getCurrentPosition());
        telemetry.addData("version","36");
        telemetry.update();


        controller.update();

        if(Math.abs(controller.left_stick_x) > Math.abs(stick_x)) {
            stick_x = controller.left_stick_x;
        } else if(Math.abs(controller.left_stick_x) < Math.abs(stick_x)) {
            stick_x = 0;
        }
        if(Math.abs(controller.left_stick_y) > Math.abs(stick_y)) {
            stick_y = controller.left_stick_y;
        } else if(Math.abs(controller.left_stick_y) < Math.abs(stick_y)) {
            stick_y = 0;
        }

        if(Math.abs(controller.left_stick_x) > 0.2 || Math.abs(controller.left_stick_y) > 0.2 ||
                Math.abs(controller.right_stick_x) > 0.2
                || controller.dpadUp() || controller.dpadDown()) {
            if(smoothing < 1) {smoothing += SMOOTH_RATE;}
        } else {
            smoothing = 0;
        }
        if(smoothing > 1) {smoothing = 1;}

        drive.drive(stick_x*speed*smoothing, stick_y*speed*smoothing, controller.right_stick_x*Math.sqrt(smoothing));


        if(controller.leftStickButtonOnce()) {slowbol = !slowbol;}
        if(slowbol) {speed=0.4;} else {speed=1;}

        if(controller.dpadRight()) {
            newgrab2.setPower(1);
            newgrab1.setPower(-1);
        } else if(controller.dpadLeft()) {
            newgrab2.setPower(-1);
            newgrab1.setPower(1);
        } else {
            newgrab2.setPower(0);
            newgrab1.setPower(0);
        }

        if(controller.dpadUp()) {lift.setDirection(DcMotor.Direction.FORWARD);lift.setPower(0.55*smoothing); armgoalposition = -1;}
        else if(controller.dpadDown()) {lift.setDirection(DcMotor.Direction.REVERSE);lift.setPower(0.4*smoothing); armgoalposition = -1;}
        else if(armgoalposition == -1) {lift.setPower(0);}

        if(controller.leftBumperOnce()) {
        }
        
        if(controller.B()) {duckDumpy.setDirection(DcMotorSimple.Direction.FORWARD);duckDumpy.setPower(0.2);
        }
        else if(controller.Y()) {duckDumpy.setDirection(DcMotorSimple.Direction.REVERSE);duckDumpy.setPower(0.2);
}
        else{duckDumpy.setPower(0);}/*newgrab1.setPower(0);newgrab2.setPower(0);}*/
        telemetry.update();
        if(controller.XOnce()) {telemetry.speak("que deporte te gusta me gusta el beisbol. ");}
    }

}

// removed