package org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Controller;


@TeleOp(name = "Wrist Test Iuli")
public class WristServoPositionTest extends OpMode {

    public ServoEx flipLeft;
    public ServoEx flipRight;
    public ServoEx wrist;
    private Controller controller;
    int flipLeftPos = 0;
    int flipRightPos = 180;
    int wristPos = 0;

    public WristServoPositionTest() {
    }

    public void init() {
        controller = new Controller(gamepad1);
        flipLeft = new SimpleServo(hardwareMap,"flipLeft", 0, 360);
        flipRight = new SimpleServo(hardwareMap, "flipRight", 0, 360);
        wrist = new SimpleServo(hardwareMap, "wrist", 0, 360);
    }

    @Override
    public void loop() {
        controller.update();
//        if (controller.dpadUp()) {
//            flipLeftPos = flipLeftPos + 10;
//            telemetry.addData("dpadUp", 1);
//        } else if (controller.dpadRight()) {
//            flipLeftPos = flipLeftPos +1;
//        } else if (controller.dpadDown()) {
//            flipLeftPos = flipLeftPos - 10;
//        } else if (controller.dpadLeft()) {
//            flipLeftPos = flipLeftPos - 1;
//        }
//        if (controller.Y()) {
//            flipRightPos = flipRightPos + 10;
//        } else if (controller.B()) {
//            flipRightPos = flipRightPos +1;
//        } else if (controller.A()) {
//            flipRightPos = flipRightPos - 10;
//        } else if (controller.X()) {
//            flipRightPos = flipRightPos - 1;
//        }
//        if (controller.rightBumper()) {
//            wristPos = wristPos + 10;
//        } else if (controller.rightStickButton()) {
//            wristPos = wristPos + 1;
//        } else if (controller.leftBumper()) {
//            wristPos = wristPos - 10;
//        } else if (controller.leftStickButton()) {
//            wristPos = wristPos - 1;
//            }
//        flipLeft.turnToAngle(flipLeftPos);
//        flipRight.turnToAngle(flipRightPos);
//        wrist.turnToAngle(wristPos);
//        try {
//            Thread.sleep(100);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
        if (controller.A()) {
            wrist.turnToAngle(256);
            flipLeft.turnToAngle(0);
            flipRight.turnToAngle(360);
            telemetry.addData("State", "Intake");
        } else if (controller.B()) {
            wrist.turnToAngle(21);
            flipLeft.turnToAngle(235);
            flipRight.turnToAngle(125);
            telemetry.addData("State", "Outtake");
        }
        telemetry.addData("Flip Left Pos", flipLeftPos);
        telemetry.addData("FLip Right Pos", flipRightPos);
        telemetry.addData("Wrist Servo Pos", wristPos);
        telemetry.update();
    }
}
//Wrist 256 intake 21 outtake
//Pivot Intake Flip Left 0 Flip Right 360 Outtake Flip Left 235 Flip Right 125