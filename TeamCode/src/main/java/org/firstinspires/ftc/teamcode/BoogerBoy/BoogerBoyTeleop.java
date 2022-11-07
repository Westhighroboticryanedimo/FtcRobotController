package org.firstinspires.ftc.teamcode.BoogerBoy;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.booger.SussySonar;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

import org.firstinspires.ftc.teamcode.Controller;

@TeleOp(name = "Booger Boy Teleop")
public class BoogerBoyTeleop extends OpMode {

    private BoogerBoyDrive drive;
    private Controller controller;
    private Servo grabby;
    private Gyro gyro;
    private DcMotor lift;
    private int maxliftdistance;
    private boolean slowmode;

    private double x, y;

    public SussySonar bebe;
    private ModernRoboticsI2cRangeSensor f;
    private ModernRoboticsI2cRangeSensor r;
    private ModernRoboticsI2cRangeSensor b;
    //private ModernRoboticsI2cRangeSensor l;
    private static double inchestosensoredge;
    private static double gyrooffset;
    private boolean cantrustsonar; // can you trust the sonar readings? if not, use boogerdometry　（太股） readings.
    private double maximumpermissibleerror;
    public Boogerdometry 太股;
    public DcMotor fl, fr, br, bl;

    @Override
    public void init() {
        slowmode = false;
        drive = new BoogerBoyDrive(this,hardwareMap);
        drive.setup();
        gyro = new Gyro(hardwareMap, false);
        gyro.reset();
        controller = new Controller(gamepad1);

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabby = hardwareMap.get(Servo.class, "grabby");
        //grabby = hardwareMap.get(CRServo.class, "grabby");
        maxliftdistance = 8964; // oct 21: changed from neg to pos nov 2: reverted

        bebe = new SussySonar();
        bebe.setup();
        bebe.denoteFieldDimensions(740,750);
        inchestosensoredge = 2; // distance between edge of sensor and midpoint, to be subtracted from measurement
        gyrooffset = 90; // make sure 90 degrees means 'forwards'
        f = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontsensor");
        r = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightsensor");
        b = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "backsensor");
        //l = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftsensor");

        gyro = new Gyro(hardwareMap,false);
        gyro.reset();

        太股 = new Boogerdometry();
        太股.init();
        fl = hardwareMap.get(DcMotor.class,"frontLeft");
        fr = hardwareMap.get(DcMotor.class,"frontRight");
        bl = hardwareMap.get(DcMotor.class,"backLeft");
        br = hardwareMap.get(DcMotor.class,"backRight");

        maximumpermissibleerror = 99999; // :)

        // init position :
        x = bebe.fieldwidth/2; y = bebe.fieldheight/2;
    }

    @Override
    public void loop() {

        controller.update();
        telemetry.addData("rotation: ", gyro.getAngleDegrees());
        telemetry.addData(" ", "booger boy activate");

        if (slowmode) {
            drive.drive(0.5 * controller.left_stick_y, 0.5 * controller.left_stick_x, 0.5 * controller.right_stick_x);
        } else {
            drive.drive(controller.left_stick_y, controller.left_stick_x, controller.right_stick_x);
        }

        if (controller.dpadUp() && lift.getCurrentPosition() < maxliftdistance) { // direction: up
            //lift.setDirection(DcMotor.Direction.FORWARD);
            telemetry.addData("", "dpadup");
            lift.setPower(1);
        } else if (controller.dpadDown() && lift.getCurrentPosition() > 0) { // direction: down
            //lift.setDirection(DcMotor.Direction.REVERSE);
            telemetry.addData("", "dpaddown");
            lift.setPower(-1);
        } else {
            lift.setPower(0);
        }

        if (controller.BOnce()) {
            grabby.setPosition(0);
            //grabby.setDirection(CRServo.Direction.REVERSE);
            //grabby.setPower(0.5);
        }
        if (controller.AOnce()) {
            grabby.setPosition(1);
            //grabby.setDirection(CRServo.Direction.FORWARD);
            //grabby.setPower(0.5);
        }
        if (controller.XOnce()) {
            slowmode = !slowmode;
        }
        telemetry.addData("servo value:", grabby.getPosition());
        telemetry.addData("lift value:", lift.getCurrentPosition());


        // las cosas del bebe sónar

        double frontdist = f.getDistance(DistanceUnit.INCH) + inchestosensoredge;
        double rightdist = r.getDistance(DistanceUnit.INCH) + inchestosensoredge;
        double backdist = b.getDistance(DistanceUnit.INCH) + inchestosensoredge;
        double leftdist = bebe.fieldwidth-rightdist;
        //double leftdist = l.getDistance(DistanceUnit.INCH) + inchestosensoredge;
        bebe.update(bebe.rotateAngle(gyro.getAngleDegrees(), gyrooffset), frontdist * 10, rightdist * 10, (backdist * 10), bebe.fieldwidth - (rightdist * 10));

        //char[][] mapImage = bebe.getMapImage(bebe.botx, bebe.boty, gyro.getAngleDegrees()+gyrooffset, 14); // minimap
        char[][] mapImage = bebe.getMapImage(x, y, bebe.rotateAngle(gyro.getAngleDegrees(),gyrooffset), 14); // minimap

        for (int my = 0; my < mapImage[0].length; my++) { // add minimap to telemetry
            String mapLine = "";
            for (int mx = 0; mx < mapImage.length; mx++) {
                mapLine = mapLine + mapImage[mx][my] + "     ";//mapImage[mx][my];
            }
            telemetry.addData("|", mapLine + " " + my);
        }
        telemetry.addData("sonar x: ", bebe.x());
        telemetry.addData("sonar y: ", bebe.y());
        telemetry.addData("front: ", f.getDistance(DistanceUnit.INCH));
        telemetry.addData("right: ", r.getDistance(DistanceUnit.INCH));
        telemetry.addData("back: ", b.getDistance(DistanceUnit.INCH));
        telemetry.addData("bot x: ", x);
        telemetry.addData("bot y: ", y);
        //telemetry.addData("dometry x:", 太股.x);
        //telemetry.addData("dometry y:", 太股.y);
        telemetry.addData("sonar trust?: ", cantrustsonar);

        // fin

        // 太股の事

        太股.updateposition(fr.getCurrentPosition(),fl.getCurrentPosition(),br.getCurrentPosition(),bl.getCurrentPosition(),gyro.getAngleDegrees());

        // 終わり
        /*
        if(bebe.distance(太股.x*10,太股.y*10,bebe.botx,bebe.boty) > maximumpermissibleerror) { // if the readings of bebe and 太股 are too far apart, rely on 太股. else, adjust 太股 to match bebe.
            cantrustsonar = false;
            x = 太股.x;
            y = 太股.y;
        } else {
            cantrustsonar = true;
            太股.x = bebe.botx;
            太股.y = bebe.boty;
            x = bebe.botx;
            y = bebe.boty;
        }
        */
        telemetry.update();
    }
}
