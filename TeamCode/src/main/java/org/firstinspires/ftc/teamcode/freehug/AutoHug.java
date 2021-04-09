package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="autohug")
public class AutoHug extends LinearOpMode {
    o_d_o_m_e_t_r_y odometry;
    Freehugdrive drive;
    static double SHOOTER_CALIBRATION = 0.375;
    GrabberFree grabber;
    private DcMotor shooterL;
    private DcMotor shooterR;
    private double current_x;
    private double current_y;
    private double close_enough = 1;

    public void go_definite(double x, double y) {
        go_relative(x-current_x,y-current_y);
    }
    public void go_relative(double x_add, double y_add) {
        double x_start = odometry.robot_x;
        double y_start = odometry.robot_y;
        double x_moved = 0;
        double y_moved = 0;
        while(Math.abs(x_moved-x_add)<close_enough) {
            drive.drive(0.40,0,0);
            x_moved = odometry.give_me_the_X()-x_start;
        }
        drive.drive(0,0,0);
        sleep(20);
        while(Math.abs(y_moved-y_add)<close_enough) {
            drive.drive(0,0.40,0);
            y_moved = odometry.give_me_the_Y()-y_start;
        }
        drive.drive(0,0,0);
        sleep(10);
    }

    public double calculateshooterpowerbasedonbatterypower() {return (0.0268 * (drive.getVoltage(hardwareMap)*drive.getVoltage(hardwareMap))) - (0.734 * drive.getVoltage(hardwareMap)) + 5.0128 + SHOOTER_CALIBRATION; }

    public void lowerArmAuto(int ms) {grabber.lowerHand();sleep(ms);grabber.restElbow(); }
    public void raiseArmAuto(int ms) { grabber.handInTheAir();sleep(ms);grabber.restElbow(); }

    public void openWristAuto(int ms) { grabber.tiltHandUp();sleep(ms);grabber.restWrist(); }

    public void closeWristAuto(int ms) {grabber.tiltHandDown();sleep(ms);grabber.restWrist(); }

    public void openHandAuto() { grabber.openHand(); }
    public void closeHandAuto() { grabber.closeHand(); }

    public void spinFlies() { shooterL.setPower(calculateshooterpowerbasedonbatterypower()); shooterR.setPower(0.92*calculateshooterpowerbasedonbatterypower());}
    public void restFlies() { shooterL.setPower(0); shooterR.setPower(0);}

    @Override
    public void runOpMode() throws InterruptedException {
        odometry = new o_d_o_m_e_t_r_y();
        drive = new Freehugdrive(this, hardwareMap);
        WebcamFree webcam = new WebcamFree(this, hardwareMap);
        grabber = new GrabberFree(this, hardwareMap);
        IntakeFree intake = new IntakeFree(this, hardwareMap);
        shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        shooterL.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        odometry.robot_position(hardwareMap.get(DcMotor.class,"odo_vert_L"),hardwareMap.get(DcMotor.class,"odo_vert_R"),hardwareMap.get(DcMotor.class,"odo_horiz"), 307.699557,50);
        odometry.recalibrate_position();
        odometry.running = true;
        odometry.run();
        
        ElapsedTime runtime = new ElapsedTime();

        telemetry.addData("Init", "ihfe");

        waitForStart();

        telemetry.addData("Helkfhai;", "fkje");
        telemetry.update();

        // Get how many rings are stacked
        int rings = 0;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {

            rings = webcam.getNumRings();

            telemetry.addData("Num rings", webcam.getNumRings());
            telemetry.addData("Ring position", webcam.getInternalRingNum());
            telemetry.update();

        }

        spinFlies();
        //move up to shooting distance
        //the actual measurement is 68 inches
        //drive.move(0.43,35,0);
        go_relative(0,35);
        //drive.move(0.43,10,90);
        go_relative(10,0);

        //move intake
        intake.intake(false,true);
        //sleep(400);
        sleep(900);
        intake.intake(false,false);

        drive.move(0.40,5,90);
        //go_relative(5,0);

        sleep(3200);
        //move intake
        intake.intake(false,true);
        sleep(1000);
        restFlies();
        intake.intake(false,false);

        if (rings == 0) {
            drive.move(0.40,35,0);
            //go_relative(0,35);
            drive.move(0.40,32,270);
            //go_relative(-32,0);
            lowerArmAuto(4500);
            openHandAuto();
        } else if (rings == 1) {
            drive.move(0.40,58,0);
            //go_relative(0,58);
            lowerArmAuto(4500);
            openHandAuto();
            drive.move(0.40,23,180);
            //go_relative(0,-23);
        } else {
            drive.move(0.40,81,0);
            //go_relative(0,81);
            drive.move(0.40,32,270);
            //go_relative(-32,0);
            lowerArmAuto(4500);
            openHandAuto();
            drive.move(0.40,46,180);
            //go_relative(0,-46);

        }
    }
}
