package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.drive.AcmeHolonomicDrive;

@Config
@Autonomous(group = "Test")
@Disabled
public class FollowerPIDTuner extends LinearOpMode {

    public static double DISTANCE = 48;

    @Override
    public void runOpMode() throws InterruptedException {

        AcmeHolonomicDrive drive = new AcmeHolonomicDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {

            Trajectory traj = drive.trajectoryBuilder(startPose)
                    .forward(DISTANCE)
                    .build();
            drive.followTrajectory(traj);
            drive.turn(Math.toRadians(90));

            startPose = traj.end().plus(new Pose2d(0, 0, Math.toRadians(90)));

        }

    }

}
