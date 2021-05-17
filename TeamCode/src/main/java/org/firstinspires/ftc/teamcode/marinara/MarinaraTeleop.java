package org.firstinspires.ftc.teamcode.marinara;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.hardware.drive.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.marinara.hardware.Grabber;
import org.firstinspires.ftc.teamcode.marinara.hardware.Intake;
import org.firstinspires.ftc.teamcode.marinara.hardware.Shooter;
import org.firstinspires.ftc.teamcode.marinara.hardware.MarinaraDrive;
import org.firstinspires.ftc.teamcode.marinara.hardware.Webcam;

@Disabled
@TeleOp(name = "Marinara: TeleOp", group = "Marinara")
public class MarinaraTeleop extends OpMode {

    // Objects
    private Intake intake;
    private Shooter shooter;
    private Grabber grabber;
    private MarinaraDrive drive;
    private Webcam webcam;

    // Odometry
    //OdometryGlobalCoordinatePosition odometry;

    // Controller object
    private Controller controller;

    @Override
    public void init() {

        // Initialize objects
        intake = new Intake(this, hardwareMap);
        shooter = new Shooter(this, hardwareMap);
        grabber = new Grabber(this, hardwareMap);
        drive = new MarinaraDrive(this, hardwareMap);
        webcam = new Webcam(this, hardwareMap);
        controller = new Controller(gamepad1);

        // Initialize odometry
        //odometry = new OdometryGlobalCoordinatePosition(hardwareMap);
        //Thread positionThread = new Thread(odometry);
        //positionThread.start();

        // Allow drive to use odometry
        //drive.getOdometry(odometry);

        // Unhook pop-out intake system
        intake.unhook();

        // Uncomment for telemetry values

        // intake.debug();
        // shooter.debug();
        // grabber.debug();
        drive.debug();
        // webcam.debug();

    }

    @Override
    public void loop() {

        // Controller
        controller.update();

        // Drive
        drive.drive(controller.left_stick_x, controller.left_stick_y, controller.right_stick_x);

        // Drive modes
        drive.togglePOV(controller.backOnce());
        drive.toggleSlow(controller.leftStickButtonOnce());

        // Reset odometry
        //odometry.resetPosition(controller.rightStickButtonOnce());

        // Shooter
        shooter.shoot(controller.Y(), drive.getVoltage(hardwareMap));

        // Intake
        intake.intake(controller.leftBumper(), controller.rightBumper());

        // Grabber
        grabber.grab(controller.XOnce());
        grabber.liftToDown(controller.start() && controller.dpadDownOnce());
        grabber.liftToUp(controller.start() && controller.dpadUpOnce());
        grabber.rotate(controller.BOnce());
        grabber.lift(controller.dpadUp() || grabber.getIsRaise(), controller.dpadDown() || grabber.getIsLower());
        grabber.update();

        // Webcam
        webcam.update();

        // Voltage
        telemetry.addData("Voltage", drive.getVoltage(hardwareMap));

        // Distance sensors
        telemetry.addData("Distance Back", drive.getDistanceBack());
        telemetry.addData("Distance Right", drive.getDistanceRight());

        // Odometry positions
        /*
        telemetry.addData("X", odometry.getX());
        telemetry.addData("Y", odometry.getY());
        telemetry.addData("Orientation", odometry.getOrientation());
        telemetry.addData("vl", odometry.getVl());
        telemetry.addData("vr", odometry.getVr());
        telemetry.addData("h", odometry.getH());
        */

        // Telemetry
        telemetry.update();

    }
/*
    @Override
    public void stop() {

        odometry.stop();

    }
*/
}
