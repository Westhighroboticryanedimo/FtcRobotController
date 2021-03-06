package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="autohug")
public class AutoHug extends LinearOpMode {
    GrabberFree grabber;
    private DcMotor shooterL;
    private DcMotor shooterR;

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

    public void openHandAuto() { grabber.openHand(); }
    public void closeHandAuto() { grabber.closeHand(); }

    public void spinFlies() { shooterL.setPower(1); shooterR.setPower(1);}
    public void restFlies() { shooterL.setPower(0); shooterR.setPower(0);}

    @Override
    public void runOpMode() throws InterruptedException {
        Freehugdrive drive = new Freehugdrive(this, hardwareMap);
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


        //WEIRD NUMBERS
        /*
        if (rings == 0) {

            // 0 ring
            drive.move(0.6,83,180);
            sleep(1000);
            drive.move(0.6, 25, -90);
        } else if (rings == 1) {

            // 1 rings
            drive.move(.6, 98, 180);
            sleep(2000);
            drive.move(.6, 16, 0);
        } else {

            // 4 rings
            drive.move(0.6,130,180);
            sleep(1000);
            drive.move(0.6, 25, -90);
            drive.move(0.6,45,0);
        }*/

        //move up to shooting distance
        //the actual measurement is 68 inches
        drive.move(0.45,58,0);
        drive.move(0.45,4,90);
        //start flywheels spinning
        spinFlies();
        sleep(1200);
        //move intake
        intake.intake(false,true);
        sleep(800);

        sleep(200);
        drive.move(0.45,1,90);

        sleep(800);
        //move intake
        intake.intake(false,true);
        sleep(2000);
        restFlies();
    }
}
