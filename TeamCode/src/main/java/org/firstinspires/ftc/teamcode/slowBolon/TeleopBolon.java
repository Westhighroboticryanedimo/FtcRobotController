package org.firstinspires.ftc.teamcode.slowBolon;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

    private boolean slowbol;
    private double speed;

    private boolean handopen;

    @Override
    public void init() {
        speed=1; handopen = true;
        slowbol = false;
        drive = new DriveBolon(this, hardwareMap);
        drive.setup();
        gyro = new Gyro(hardwareMap,false);
        controller = new Controller(gamepad1);
        gyro.reset();
        newgrab1 = hardwareMap.get(CRServo.class, "grab1");
        newgrab2 = hardwareMap.get(CRServo.class, "grab2");
        newgrab1.setDirection(CRServo.Direction.FORWARD); newgrab2.setDirection(CRServo.Direction.REVERSE);

        duckDumpy = hardwareMap.get(DcMotor.class,"duckDumpy");

        lift = hardwareMap.get(DcMotor.class,"lift");
        o = new OdometryBolon();
        o.init(hardwareMap.get(DcMotor.class,"frontLeft"));


        o.run();
    }

    @Override
    public void loop() {

        telemetry.addData("milfs", gyro.getAngleDegrees());
        telemetry.addData("version","32");

        controller.update();
        drive.drive(controller.left_stick_x*speed, controller.left_stick_y*speed, controller.right_stick_x);
        if(controller.dpadDownOnce()) {o.resetdistance();}

        if(controller.leftStickButtonOnce()) {slowbol = !slowbol;}
        if(slowbol) {speed=0.4;} else {speed=1;}

        if(controller.dpadLeft()) {
            newgrab1.setDirection(CRServo.Direction.FORWARD);newgrab2.setDirection(CRServo.Direction.REVERSE);
            newgrab1.setPower(1);
            newgrab2.setPower(1);
        }
        else if(controller.dpadRight()) {
            newgrab1.setDirection(CRServo.Direction.REVERSE);newgrab2.setDirection(CRServo.Direction.FORWARD);
            newgrab2.setPower(1);
            newgrab1.setPower(1);
        } else {
            newgrab2.setPower(0);
            newgrab1.setPower(0);
        }

        if(controller.dpadUp()) {lift.setDirection(DcMotor.Direction.FORWARD);lift.setPower(0.37);}
        else if(controller.dpadDown()) {lift.setDirection(DcMotor.Direction.REVERSE);lift.setPower(0.27);}
        else{lift.setPower(0);}

        if(controller.leftBumperOnce()) {
        }

        if(controller.B()) {duckDumpy.setDirection(DcMotorSimple.Direction.FORWARD);duckDumpy.setPower(1);
        }
        else if(controller.Y()) {duckDumpy.setDirection(DcMotorSimple.Direction.REVERSE);duckDumpy.setPower(1);
}
        else{duckDumpy.setPower(0);newgrab1.setPower(0);newgrab2.setPower(0);}
        telemetry.update();
        if(controller.XOnce()) {telemetry.speak("que deporte te gusta me gusta el beisbol");}
    }

}

// removed