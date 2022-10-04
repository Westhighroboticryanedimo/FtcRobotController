package org.firstinspires.ftc.teamcode.slowBolon;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

@Disabled
@TeleOp(name = "smol bolon teleop")
public class TeleopBolon extends OpMode {

    private DriveBolon drive;
    private Controller controller;
    private Controller boobus;
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
    private int armupposition = -3000, armdownposition = 0;
    private int armgoalposition;

    private boolean handopen;

    private Bolodometry bd;

    @Override
    public void init() {
        smoothing = 0;
        SMOOTH_RATE = 0.05;
        stick_x = 0; stick_y = 0;
        armgoalposition = -1;
        armupposition = 3000; armdownposition = 0;
        speed=1; handopen = true;
        slowbol = false;
        drive = new DriveBolon(this, hardwareMap);
        drive.setup();
        gyro = new Gyro(hardwareMap,false);
        controller = new Controller(gamepad1);
        boobus = new Controller(gamepad2);
        gyro.reset();
        newgrab1 = hardwareMap.get(CRServo.class, "grab1");
        newgrab2 = hardwareMap.get(CRServo.class, "grab2");
        // BOLODOMETRY
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
            if(armposition > armgoalposition) {lift.setPower(-1);}
            else if(armposition < armgoalposition) {lift.setPower(1);}

            if(armgoalposition == armupposition && armposition > armupposition-200) {
                armgoalposition = -1;
            }
            if(armgoalposition == armdownposition && armposition < armdownposition+200) {
                armgoalposition = -1;
            }
        }/*
        if(controller.left_trigger>0.2) {
            //telemetry.addData("PRESSING","");
            if(armgoalposition != armupposition && armgoalposition != armdownposition) {
                telemetry.addData("pressing","");
                armgoalposition = armupposition;
            }
            else if(armgoalposition >= (armupposition+armdownposition)/2) {armgoalposition = armdownposition;}
            else if(armgoalposition <= (armupposition+armdownposition)/2) {armgoalposition = armupposition;}
        }*/

        telemetry.addData("milfs", gyro.getAngleDegrees());
        telemetry.addData("Bolodometry y: ",bd.y);
        telemetry.addData("Bolodometry x: ",bd.x);
        telemetry.addData("smoothing: ", smoothing);
        telemetry.addData("FL",fl.getCurrentPosition());
        telemetry.addData("FR",fr.getCurrentPosition());
        telemetry.addData("BR",br.getCurrentPosition());
        telemetry.addData("BL",bl.getCurrentPosition());
        telemetry.addData("version","37.boopa.0");
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
            if(smoothing > 0) {smoothing -= SMOOTH_RATE*2;}
        }
        if(smoothing > 1) {smoothing = 1;}
        if(smoothing < 0) {smoothing = 0;}

        double bx = boobus.left_stick_x*0.4, by = boobus.left_stick_y*0.4, bt = boobus.right_stick_x*0.4;

        if(!slowbol) {
            drive.drive((stick_x*speed*Math.sqrt(smoothing)*0.73)+bx, (stick_y*speed*Math.sqrt(smoothing))+by, (controller.right_stick_x*Math.sqrt(smoothing))+bt);
        } else {
            drive.drive((stick_x * speed * Math.sqrt(smoothing))+bx, (stick_y * speed * smoothing)+by, (controller.right_stick_x * Math.sqrt(smoothing))+bt);
        }

        if(controller.leftStickButtonOnce()) {
            gamepad1.rumble(700);
            slowbol = !slowbol;}
        if(slowbol) {speed=0.4;} else {speed=1;}

        if(controller.dpadRight() || controller.leftBumper()) {
            newgrab2.setPower(1);
            newgrab1.setPower(-1);
        } else if(controller.dpadLeft() || controller.rightBumper()) {
            newgrab2.setPower(-1);
            newgrab1.setPower(1);
        } else {
            newgrab2.setPower(0);
            newgrab1.setPower(0);
        }

        if(controller.right_trigger>0.2) {lift.setPower(0.55); armgoalposition = -1;}
        else if(controller.left_trigger>0.2) {lift.setPower(-0.4); armgoalposition = -1;}
        else if(armgoalposition == -1) {lift.setPower(0);}
        if(controller.leftBumperOnce()) {
            if(armgoalposition != armupposition && armgoalposition != armdownposition) {
                telemetry.addData("pressing","");
                armgoalposition = armupposition;
            }
            else if(armgoalposition >= (armupposition+armdownposition)/2) {armgoalposition = armdownposition;}
            else if(armgoalposition <= (armupposition+armdownposition)/2) {armgoalposition = armupposition;}
        }
        
        if(controller.B()) {
            gamepad1.rumble(500);
            duckDumpy.setDirection(DcMotorSimple.Direction.FORWARD);duckDumpy.setPower(0.2);
        }
        else if(controller.Y()) {
            gamepad1.rumble(500);
            duckDumpy.setDirection(DcMotorSimple.Direction.REVERSE);duckDumpy.setPower(0.2);
}
        else{duckDumpy.setPower(0);}/*newgrab1.setPower(0);newgrab2.setPower(0);}*/
        telemetry.update();
        if(controller.XOnce()) {telemetry.speak("que deporte te gusta me gusta el beisbol. ");}

        // BOOBUS

        boobus.update();

        if(boobus.left_trigger > 0.2) {lift.setPower(-0.6);}
        if(boobus.right_trigger > 0.2) {lift.setPower(0.6);}

        if(boobus.A()) {duckDumpy.setPower(1);}
        if(boobus.B()) {duckDumpy.setPower(-1);}

        if(boobus.dpadDown()) {gamepad2.rumble(500); gamepad1.rumble(500);}

        // BOOBUS
    }

}

// removed