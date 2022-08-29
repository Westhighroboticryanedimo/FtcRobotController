package org.firstinspires.ftc.teamcode.reee;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="reee teleop", group="reee")

public class reeeTele extends OpMode {
    private ModernRoboticsI2cRangeSensor range;

    private DcMotor motor;
    private Servo rotator;
    private Controller controller;
    private double pos = 0.0;
    // private static double TURN_TICKS = 25.95;

    boolean dir = false;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        rotator = hardwareMap.get(Servo.class, "rotator");
        // controller = new Controller(gamepad1);
        // RANGE1 = hardwareMap.i2cDevice.get("range");
        // range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
    }

    @Override
    public void loop() {
        telemetry.addData("encoder:", motor.getCurrentPosition());
        telemetry.update();
//        ElapsedTime runtime = new ElapsedTime();
//        if (motor.getCurrentPosition() <= 0) {
//            dir = true;
//        } else if (motor.getCurrentPosition() >= TURN_TICKS) {
//            dir = false;
//        }
//
//        if (dir) {
//            motor.setPower(0.25);
//        } else {
//            motor.setPower(-0.25);
//        }
//        telemetry.addData("ticks:", motor.getCurrentPosition());
//        telemetry.addData("ultrasonic:", range.rawUltrasonic());
//        telemetry.addData("optical:", range.rawOptical());
//        telemetry.addData("range:", range.getDistance(DistanceUnit.INCH));
//        telemetry.addData("a:", range.aParam);
//        telemetry.addData("b:", range.bParam);
//        telemetry.addData("c:", range.cParam);
//        telemetry.addData("d:", range.dParam);
//        telemetry.update();
//        rotator.setPosition(0.0);
//        telemetry.addData("0", 0);
//        while (runtime.seconds() < 2) {
//
//        }
//        rotator.setPosition(0.5);
//        telemetry.addData("90", 90);
//        while (runtime.seconds() < 6) {
//
//        }
//        if (controller.AOnce()) {
//            pos = 0.0;
//        } else if (controller.BOnce()) {
//            pos = 0.5;
//        } else if (controller.leftBumperOnce()) {
//            pos -= 0.05;
//        } else if (controller.rightBumperOnce()) {
//            pos += 0.05;
//        }
//        rotator.setPosition(pos);
    }

}
