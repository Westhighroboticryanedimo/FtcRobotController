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
    private Gyro gyro;
    private OdometryBolon o;
    private CRServo newgrab1, newgrab2;

    private DcMotor duckDumpy;
    private DcMotor lift;

    DcMotor fl, fr, br, bl;

    private boolean slowbol;
    private double speed;
    private double armposition;
    private int armupposition = 3000, armdownposition = 0;
    private int armgoalposition;

    private boolean handopen;

    @Override
    public void init() {
        armgoalposition = -1;
        armupposition = 3000; armdownposition = 0;
        speed=1; handopen = true;
        slowbol = false;
        drive = new DriveBolon(this, hardwareMap);
        drive.setup();
        gyro = new Gyro(hardwareMap,false);
        controller = new Controller(gamepad1);
        gyro.reset();
        newgrab1 = hardwareMap.get(CRServo.class, "grab1");
        newgrab2 = hardwareMap.get(CRServo.class, "grab2");
        //newgrab1.setDirection(CRServo.Direction.FORWARD); newgrab2.setDirection(CRServo.Direction.REVERSE);

        duckDumpy = hardwareMap.get(DcMotor.class,"duckDumpy");

        lift = hardwareMap.get(DcMotor.class,"lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        o = new OdometryBolon();
        o.init(hardwareMap.get(DcMotor.class,"frontLeft"));



        o.run();

        fl = hardwareMap.get(DcMotor.class,"frontLeft");
        fr = hardwareMap.get(DcMotor.class,"frontRight");
        bl = hardwareMap.get(DcMotor.class,"backLeft");
        br = hardwareMap.get(DcMotor.class,"backRight");
    }

    @Override
    public void loop() {
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
        telemetry.addData("FL",fl.getCurrentPosition());
        telemetry.addData("FR",fr.getCurrentPosition());
        telemetry.addData("BR",br.getCurrentPosition());
        telemetry.addData("BL",bl.getCurrentPosition());
        telemetry.addData("version","34");
        telemetry.update();


        controller.update();
        drive.drive(controller.left_stick_x*speed, controller.left_stick_y*speed, controller.right_stick_x);
        if(controller.dpadDownOnce()) {o.resetdistance();}

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

        if(controller.dpadUp()) {lift.setDirection(DcMotor.Direction.FORWARD);lift.setPower(0.55); armgoalposition = -1;}
        else if(controller.dpadDown()) {lift.setDirection(DcMotor.Direction.REVERSE);lift.setPower(0.4); armgoalposition = -1;}
        else if(armgoalposition == -1) {lift.setPower(0);}

        if(controller.leftBumperOnce()) {
        }

        if(controller.B()) {duckDumpy.setDirection(DcMotorSimple.Direction.FORWARD);duckDumpy.setPower(0.2);
        }
        else if(controller.Y()) {duckDumpy.setDirection(DcMotorSimple.Direction.REVERSE);duckDumpy.setPower(0.2);
}
        else{duckDumpy.setPower(0);}/*newgrab1.setPower(0);newgrab2.setPower(0);}*/
        telemetry.update();
        if(controller.XOnce()) {telemetry.speak("que deporte te gusta me gusta el beisbol. AYUDAME NECESITO AYUDO PORQUE LOS AVES HACE LOS COSAS MAS HORIBLES QUE PUEDES COMPRENDER!!!");}
    }

}

// removed