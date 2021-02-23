package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.marinara.hardware.Grabber;

@TeleOp(name = "Free Hugs teleop")
public class Freehugteleop extends OpMode {
    private Freehugdrive drive;
    private IntakeFree intake;
    private Controller controller;
    private DcMotor shooterL;
    private DcMotor shooterR;

    private GrabberFree grabber;
    double adjustment = 1;

    @Override
    public void init() {
        adjustment = 1;
        drive = new Freehugdrive(this, hardwareMap);
        intake = new IntakeFree(this, hardwareMap);
        controller = new Controller(gamepad1);
        shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        shooterL.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        grabber = new GrabberFree(this, hardwareMap);
        drive.debug();
    }

    @Override
    public void loop() {
        controller.update();

        drive.togglePOV(controller.backOnce());
        drive.drive(controller.left_stick_x * adjustment, -controller.left_stick_y * adjustment, controller.right_stick_x * adjustment);
        intake.intake(controller.B(), controller.A());

        if (controller.X()) {

            shooterL.setPower(1);
            shooterR.setPower(1);

        } else {

            shooterL.setPower(0);
            shooterR.setPower(0);

        }

        if (controller.leftStickButtonOnce()) {
            if(adjustment == 1) {
                adjustment = 0.5;
            }
            else{
                adjustment = 1;
            }
        }

        /*
        if controller (button for lifting arm)
            lift arm()

        if controller (button for lowering arm)
            lower arm()

         if controller (button for closing wrist)
            close wrist()

         if controller (button for opening wrist)
            open wrist()

         if controller (button for opening hand)
            open hand()

         if controller (button for closing hand)
            close hand()
         */

    }
}
