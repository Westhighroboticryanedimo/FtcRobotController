package org.firstinspires.ftc.teamcode.slowBolon;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name = "Balloon Mode")
public class AutoBolon extends LinearOpMode{

    private org.firstinspires.ftc.teamcode.slowBolon.CamBolon Cam;
    private int DONDEESTAELDUCKY;
    private OdometryBolon o;
    private DriveBolon d;

    double distance;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Cam = new org.firstinspires.ftc.teamcode.slowBolon.CamBolon();
        //Cam.init(hardwareMap);

        OdometryBolon o = new OdometryBolon();
        o.init(hardwareMap.get(DcMotor.class,"frontRight"));
        hardwareMap.get(DcMotor.class,"frontRight").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.get(DcMotor.class,"frontRight").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        distance = o.distance;

        o.run();
        d = new DriveBolon(this,hardwareMap);
        //DONDEESTAELDUCKY = Cam.getspot();
        ElapsedTime runtime = new ElapsedTime();

        /*telemetry.addData("where",DONDEESTAELDUCKY);
        telemetry.addData("leastduckydiff", Cam.pipeline.leastduckydiff);
        telemetry.addData("leasttapediff", Cam.pipeline.leasttapediff);*/
        telemetry.addData("distance",o.distance);
        telemetry.update();

        /*if(DONDEESTAELDUCKY <= 1) {

        }

        else if(DONDEESTAELDUCKY <= 2) {

        }

        else {

        }

        while(runtime.seconds() < 30) {

        }*/

        while(distance < 1184) {
            d.drive(0,-0.4,0);
            o.updatedistance();
            distance=o.distance;
            telemetry.addData("distancee",o.distance);
            telemetry.update();
        }
        o.resetdistance(); distance=o.distance;
        while(distance < 1900) {
            d.drive(-0.4,0,0);
            o.updatedistance();
            distance=o.distance;
            telemetry.addData("dostance",o.distance);
            telemetry.update();
        }
    }
}
