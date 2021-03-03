package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controller;
import org.firstinspires.ftc.teamcode.marinara.hardware.Grabber;

import static android.os.SystemClock.sleep;


@TeleOp(name = "Free Hugs teleop")
public class Freehugteleop extends OpMode {
    private Freehugdrive drive;
    private IntakeFree intake;
    private Controller controller;
    private DcMotor shooterL;
    private DcMotor shooterR;
    private FreeReturn freeReturn;

    private GrabberFree grabber;
    double adjustment = 1;

    @Override
    public void init() {
        //adjustment = 1;//
        drive = new Freehugdrive(this, hardwareMap);
        intake = new IntakeFree(this, hardwareMap);
        controller = new Controller(gamepad1);
        shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        shooterL.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        grabber = new GrabberFree(this, hardwareMap);
        freeReturn = new FreeReturn();
        drive.debug();

        freeReturn.xOffset = 0;
        freeReturn.yOffset = 0;
        freeReturn.robotCurrentAngle = 0;
    }

    @Override
    public void loop() {
        controller.update();

        drive.togglePOV(controller.backOnce());
        drive.drive(controller.left_stick_x * adjustment, controller.left_stick_y * adjustment, controller.right_stick_x * adjustment);

        if(drive.isInPOVMode()) {

            freeReturn.updateAngle(controller.right_stick_x * adjustment);
            freeReturn.updateOffsets(controller.left_stick_x * adjustment, controller.left_stick_y * adjustment);

        } else if(drive.isInPOVMode() == false) {

            freeReturn.updateAngle(0);
            freeReturn.updateOffsets(controller.left_stick_x * adjustment, controller.left_stick_y * adjustment);

        }

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

        if(controller.right_trigger >= 0.2) {
            grabber.handInTheAir();
        }
        else if(controller.left_trigger >= 0.2) {
            grabber.lowerHand();
        }
        else {
            grabber.restElbow();
        }

        /*//position lock and return commands
        if(controller.dpadUpOnce()) {
            lockPosition();
        }
        else if(controller.dpadDown()) {
            returnToPosition();
        }*/

        //grabber hand open / close
        if(controller.dpadRightOnce()) {
            grabber.openHand();
        }
        else if(controller.dpadLeftOnce()) {
            grabber.closeHand();
        }

        //dpad: UP to lock position , DOWN to return to position
        if(controller.dpadUpOnce()) {
            freeReturn.lockPosition();
        } else if(controller.dpadDownOnce()) {
            freeReturn.freelyReturn();
        }

    }
}
