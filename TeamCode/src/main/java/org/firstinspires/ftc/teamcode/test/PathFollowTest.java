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
public class PathFollowTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        AcmeHolonomicDrive drive = new AcmeHolonomicDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(0,0,0))
                .splineToConstantHeading(new Pose2d(-30, -30, 0))
                .splineTo(new Pose2d(0, -30, 1.5707963267948966))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

    }

}