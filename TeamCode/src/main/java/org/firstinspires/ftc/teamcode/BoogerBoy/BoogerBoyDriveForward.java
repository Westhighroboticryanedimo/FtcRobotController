package org.firstinspires.ftc.teamcode.BoogerBoy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name = "Drive Forward Autonomous")
public class BoogerBoyDriveForward extends LinearOpMode {
    private int drivedistance;
    private BoogerBoyDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        drive = new BoogerBoyDrive(this, hardwareMap);
        drive.debug();
        drive.move(0.4,555,0);
    }
}
