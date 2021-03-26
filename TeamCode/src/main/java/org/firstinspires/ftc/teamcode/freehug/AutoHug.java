package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="autohug")
public class AutoHug extends LinearOpMode {
    Freehugdrive drive;
    static double SHOOTER_CALIBRATION = 0.3745;
    GrabberFree grabber;
    private DcMotor shooterL;
    private DcMotor shooterR;

    public double calculateshooterpowerbasedonbatterypower() {
        return (0.0268 * (drive.getVoltage(hardwareMap)*drive.getVoltage(hardwareMap))) - (0.734 * drive.getVoltage(hardwareMap)) + 5.0128 + SHOOTER_CALIBRATION;
    }

    public void lowerArmAuto(int ms) {
        grabber.lowerHand();
        sleep(ms);
        grabber.restElbow();
    }
    public void raiseArmAuto(int ms) {
        grabber.handInTheAir();
        sleep(ms);
        grabber.restElbow();
    }

    public void openWristAuto(int ms) {
        grabber.tiltHandUp();
        sleep(ms);
        grabber.restWrist();
    }

    public void closeWristAuto(int ms) {
        grabber.tiltHandDown();
        sleep(ms);
        grabber.restWrist();
    }

    public void openHandAuto() { grabber.openHand(); }
    public void closeHandAuto() { grabber.closeHand(); }

    public void spinFlies() { shooterL.setPower(calculateshooterpowerbasedonbatterypower()); shooterR.setPower(0.92*calculateshooterpowerbasedonbatterypower());}
    public void restFlies() { shooterL.setPower(0); shooterR.setPower(0);}

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Freehugdrive(this, hardwareMap);
        WebcamFree webcam = new WebcamFree(this, hardwareMap);
        grabber = new GrabberFree(this, hardwareMap);
        IntakeFree intake = new IntakeFree(this, hardwareMap);
        shooterL = hardwareMap.get(DcMotor.class, "shooterL");
        shooterR = hardwareMap.get(DcMotor.class, "shooterR");
        shooterL.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        // Get how many rings are stacked
        int rings = 0;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {

            rings = webcam.getNumRings();

            telemetry.addData("Num rings", webcam.getNumRings());
            telemetry.addData("Ring position", webcam.getInternalRingNum());
            telemetry.update();

        }

/*
        //move up to shooting distance
        //the actual measurement is 68 inches
        drive.move(0.43,35,0);
        drive.move(0.43,10,90);
*/
        //start flywheels spinning
        spinFlies();
        sleep(4500);
        //move intake
        intake.intake(false,true);
        //sleep(400);
        sleep(4000);
/*
        drive.move(0.40,5,90);

        sleep(8000);
        //move intake
        intake.intake(false,true);
        sleep(800);
        restFlies();
        intake.intake(false,false);

        if (rings == 0) {
            drive.move(0.40,35,0);
            drive.move(0.40,32,270);
            lowerArmAuto(4500);
            openHandAuto();
        } else if (rings == 1) {
            drive.move(0.40,58,0);
            lowerArmAuto(4500);
            openHandAuto();
            drive.move(0.40,23,180);
        } else {
            drive.move(0.40,81,0);
            drive.move(0.40,32,270);
            lowerArmAuto(4500);
            openHandAuto();
            drive.move(0.40,46,180);

        }*/
    }
}
