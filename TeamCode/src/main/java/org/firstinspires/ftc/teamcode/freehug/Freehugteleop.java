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
    private FreeReturn freeReturn;
    private boolean fullpower;
    private boolean shooting;
    private o_d_o_m_e_t_r_y odometry;

    private GrabberFree grabber;
    double adjustment = 0.07;

    //TO BE ADJUSTED MANUALLY
    static double DISTANCE_CALIBRATION = 1.7;
    static double TIME_CALIBRATION = 50;
    static double ANGLE_CALIBRATION = 20;
    //DONT CHANGE THIS ONE
    static double SHOOTER_CALIBRATION = 0.3745;

    @Override
    public void init() {
        fullpower = false;
        adjustment = 1;
        drive = new Freehugdrive(this, hardwareMap);
        intake = new IntakeFree(this, hardwareMap);
        controller = new Controller(gamepad1);
        shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        shooterL.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        grabber = new GrabberFree(this, hardwareMap);
        freeReturn = new FreeReturn();
        odometry = new o_d_o_m_e_t_r_y();
        odometry.robot_position(hardwareMap.get(DcMotor.class,"odo_vert_L"),hardwareMap.get(DcMotor.class,"odo_vert_R"),hardwareMap.get(DcMotor.class,"odo_horiz"), 307.699557,50);
        odometry.recalibrate_position();
        odometry.running = true;
        odometry.run();
        drive.debug();

        freeReturn.xOffset = 0;
        freeReturn.yOffset = 0;
        freeReturn.robotCurrentAngle = 0;
    }
/*
    public void freelyReturn() {
        double xo = freeReturn.xOffset * DISTANCE_CALIBRATION;
        double yo = freeReturn.yOffset * DISTANCE_CALIBRATION;

        freeReturn.freely_hugging = true;
        //do things
        if(drive.isPOVMode()) {
            drive.togglePOV(true);
        }

        if(yo > 0) {
            drive.drive(0,-0.6,0);
        } else if(yo < 0) {
            drive.drive(0,0.6,0);
        }
        sleep((int)(Math.abs(yo) * TIME_CALIBRATION));
        drive.drive(0,0,0);

        if(xo > 0) {
            drive.drive(-0.6,0,0);
        } else if(xo < 0) {
            drive.drive(0.6,0,0);
        }
        sleep((int)(Math.abs(xo)*TIME_CALIBRATION));
        drive.drive(0,0,0);

        sleep(200);
        freeReturn.lockPosition();
        freeReturn.freely_hugging = false;
    }*/

    public double calculateshooterpowerbasedonbatterypower() {
        return (0.0268 * (drive.getVoltage(hardwareMap)*drive.getVoltage(hardwareMap))) - (0.734 * drive.getVoltage(hardwareMap)) + 5.0128 + SHOOTER_CALIBRATION;
    }

    @Override
    public void loop() {
        if(!fullpower) {
            intake.rightPower = calculateshooterpowerbasedonbatterypower();
            intake.leftPower = calculateshooterpowerbasedonbatterypower();
        } else if(fullpower) {
            intake.rightPower = 1;
            intake.leftPower = 1;
        }
        controller.update();

        drive.togglePOV(controller.backOnce());

        if(!freeReturn.freely_hugging) {
            drive.drive(controller.left_stick_x * adjustment, controller.left_stick_y * adjustment, controller.right_stick_x * adjustment);

            if (drive.isPOVMode()) {

                freeReturn.updateAngle(controller.right_stick_x * adjustment * ANGLE_CALIBRATION);
                freeReturn.updateOffsets(controller.left_stick_x * adjustment, controller.left_stick_y * adjustment);

            } else if (!drive.isPOVMode()) {

                //yes, updateAngle(0) MUST be here
                freeReturn.updateAngle(0);
                freeReturn.updateOffsets(controller.left_stick_x * adjustment, controller.left_stick_y * adjustment);

            }
        }

        intake.intake(controller.B(), controller.A());

        if (controller.XOnce()) {
            if(shooting) {
                shooting = false;
            } else{
                shooting = true;
            }
        }
        if(shooting) {shooterL.setPower(intake.leftPower);shooterR.setPower(intake.rightPower*0.92);} else{shooterL.setPower(0);shooterR.setPower(0);}

        if (controller.leftStickButtonOnce()) {
            if(adjustment == 0.7) {
                adjustment = 0.4;
            }
            else{
                adjustment = 0.7;
            }
        }

        if(controller.rightStickButtonOnce()) {
            if(fullpower == false) {
                fullpower = true;
            } else{
                fullpower = false;
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
        if(controller.leftBumperOnce()) {
            odometry.recalibrate_position();
        }
        else if(controller.rightBumperOnce()) {
            freeReturn.freely_hugging = true;

            double x_start = odometry.robot_x;
            double y_start = odometry.robot_y;
            double x_moved = 0;
            double y_moved = 0;
            while(Math.abs(x_moved)<1) {
                drive.drive(0.40,0,0);
                x_moved = odometry.give_me_the_X()-x_start;
            }
            drive.drive(0,0,0);
            sleep(20);
            while(Math.abs(y_moved)<1) {
                drive.drive(0,0.40,0);
                y_moved = odometry.give_me_the_Y()-y_start;
            }
            drive.drive(0,0,0);
            sleep(10);
            freeReturn.freely_hugging = false;
        }



        //grabber hand open / close
        if(controller.dpadRightOnce()) {
            grabber.openHand();
        }
        else if(controller.dpadLeftOnce()) {
            grabber.closeHand();
        }

        //dpad: UP to lock position , DOWN to return to position
        if(controller.dpadUp()) {
            grabber.tiltHandUp();
        } else if(controller.dpadDown()) {
            grabber.tiltHandDown();
        } else{
            grabber.restWrist();
        }

    }
}