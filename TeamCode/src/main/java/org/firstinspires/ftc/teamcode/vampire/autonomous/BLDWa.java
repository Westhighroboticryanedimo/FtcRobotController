package org.firstinspires.ftc.teamcode.vampire.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vampire.hardware.Arm;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.hardware.VampireDrive;
import org.firstinspires.ftc.teamcode.vampire.hardware.Webcam;
import org.firstinspires.ftc.teamcode.vampire.roadrunner.drive.VampireRRDrive;

@Autonomous(name="Vampire: BLDWa", group="Vampire")
public class BLDWa extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
/*
        // Subsystems
        VampireRRDrive drive = new VampireRRDrive(hardwareMap);
        Arm arm = new Arm(this, hardwareMap);
        Intake intake = new Intake(this, hardwareMap);

        // Set starting position
        Pose2d startPose = new Pose2d(3.25, 63.75, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        // Create trajectories
        Trajectory deploy = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0, 40), Math.toRadians(125))
                .build();
        Trajectory warehouse = drive.trajectoryBuilder(deploy.end(), true)
                .splineTo(new Vector2d(60, -40), 180)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        // Deploy cargo
        drive.followTrajectory(deploy);
        arm.setLift(3);
        sleep(1000);
        intake.reverse();
        sleep(2000);
        intake.stop();

        // Go to warehouse
        drive.followTrajectory(warehouse);
*/

        // Dum auto RIP
        VampireDrive drive = new VampireDrive(this, hardwareMap);
        Arm arm = new Arm(this, hardwareMap);
        Intake intake = new Intake(this, hardwareMap);
        Webcam webcam = new Webcam(this, hardwareMap);

        // Elapsed time for timed motion
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;

        // Get how many rings are stacked
        int position = 3;
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4) {

            position = webcam.getCargoPos();
            webcam.update();
            telemetry.update();

        }

        arm.setLift(position);
        drive.move(0.5, 40, 27);
        drive.turn(0.5, 45);
        intake.reverse();
        sleep(3000);
        intake.stop();
        drive.move(0.3, 15, 180);
        drive.turn(0.5, 25);
        drive.move(1, 80, 180);

    }

}


