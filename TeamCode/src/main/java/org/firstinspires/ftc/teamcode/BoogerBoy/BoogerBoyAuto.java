package org.firstinspires.ftc.teamcode.BoogerBoy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.slowBolon.DriveBolon;

@Autonomous(name = "minicat")
public class BoogerBoyAuto extends LinearOpMode{
    private BoogerBoyDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        drive = new BoogerBoyDrive(this,hardwareMap);
        ElapsedTime runtime = new ElapsedTime();
        drive.move(10,10,10);


    }
}
