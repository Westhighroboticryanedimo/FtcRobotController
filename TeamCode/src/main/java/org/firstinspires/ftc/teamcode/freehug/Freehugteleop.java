package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Controller;

import static android.os.SystemClock.sleep;


@TeleOp(name = "Free Hugs teleop")
public class Freehugteleop extends OpMode {
    private Freehugdrive drive;
    private IntakeFree intake;
    private Controller controller;
    private DcMotor shooterL;
    private DcMotor shooterR;

    private GrabberFree grabber;
    double adjustment = 1;

    //for toggling between driver control and robot self driving
    boolean I_move_by_meself_now = false;

    //TO BE ADJUSTED MANUALLY
    final static double CALIBRATION = 1;
    final static double TIME_CALIBRATION = 10;

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
        drive.debug();
    }

    double xOffset;
    double yOffset;

    public void lockPosition() {
        xOffset = 0;
        yOffset = 0;
        I_move_by_meself_now = false;
    }
    public void updateOffsets(double xChange, double yChange) {
        xOffset += xChange;
        yOffset += yChange;
    }
    public void returnToPosition() {
        I_move_by_meself_now = true;

        yOffset = Math.abs(yOffset) * CALIBRATION;
        xOffset = Math.abs(xOffset) * CALIBRATION;

        drive.drive(0,(yOffset * -1),0);
        sleep((long) (yOffset * TIME_CALIBRATION));
        drive.drive(0,0,0);

        drive.drive(xOffset,0,0);
        sleep((long) (xOffset * TIME_CALIBRATION));
        drive.drive(0,0,0);

        I_move_by_meself_now = false;
    }
    //if the robot fails to return it's likely because of the above returnToPosition();

    @Override
    public void loop() {
        controller.update();

        drive.togglePOV(controller.backOnce());
        if(I_move_by_meself_now == false) {
            drive.drive(controller.left_stick_x * adjustment, -controller.left_stick_y * adjustment, controller.right_stick_x * adjustment);
            updateOffsets(controller.left_stick_x * adjustment, -controller.left_stick_y * adjustment);
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

        //position lock and return commands
        if(controller.dpadUpOnce()) {
            lockPosition();
        }
        else if(controller.dpadDown()) {
            returnToPosition();
        }

        //grabber hand open / close
        if(controller.dpadRightOnce()) {
            grabber.openHand();
        }
        else if(controller.dpadLeftOnce()) {
            grabber.closeHand();
        }

    }
}
