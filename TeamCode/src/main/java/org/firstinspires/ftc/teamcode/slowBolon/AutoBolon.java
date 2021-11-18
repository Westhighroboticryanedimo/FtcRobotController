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

    @Override
    public void runOpMode() throws InterruptedException {
        Cam = new org.firstinspires.ftc.teamcode.slowBolon.CamBolon();
        Cam.init(hardwareMap);
        DONDEESTAELDUCKY = Cam.getspot();

        telemetry.addData("where",DONDEESTAELDUCKY);
        telemetry.addData("leastduckydiff", Cam.pipeline.leastduckydiff);
        telemetry.addData("leasttapediff", Cam.pipeline.leasttapediff);
        telemetry.update();

        if(DONDEESTAELDUCKY <= 1) {

        }

        else if(DONDEESTAELDUCKY <= 2) {

        }

        else {

        }

    }
}
