package org.firstinspires.ftc.teamcode.vampire.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vampire.hardware.Arm;
import org.firstinspires.ftc.teamcode.vampire.hardware.DuckDuckGo;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.hardware.VampireDrive;
import org.firstinspires.ftc.teamcode.vampire.hardware.Webcam;
import org.firstinspires.ftc.teamcode.vampire.roadrunner.drive.VampireRRDrive;

@Autonomous(name="Vampire: BRDWhWa", group="Vampire")
public class BRDWhWa extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Dum auto RIP
        VampireDrive drive = new VampireDrive(this, hardwareMap);
        Arm arm = new Arm(this, hardwareMap);
        Intake intake = new Intake(this, hardwareMap);
        DuckDuckGo spin = new DuckDuckGo(this, hardwareMap);
        Webcam webcam = new Webcam(this, hardwareMap);
        webcam.debug();

        // Elapsed time for timed motion
        ElapsedTime runtime = new ElapsedTime();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Get how many rings are stacked
        int position = 3;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {

            position = webcam.getCargoPos();
            webcam.update();
            telemetry.update();

        }

        if (position == 1) {

            drive.move(0.6, 28, 0);
            drive.move(0.6, 13, -90);

        } else drive.move(0.5, 28, -33);

        arm.setLift(position);
        drive.turn(0.5, -45);
        intake.reverse();
        sleep(3000);
        intake.stop();
        drive.turn(1, 45);
        drive.move(0.5, 40, 111);
        spin.spinBlue();
        sleep(5000);
        spin.stop();
        drive.turn(1, 10);
        drive.move(0.6, 22, 20);
        arm.setLift(0);
    }

}
