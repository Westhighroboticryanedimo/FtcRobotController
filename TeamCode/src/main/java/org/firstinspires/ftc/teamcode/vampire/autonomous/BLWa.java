package org.firstinspires.ftc.teamcode.vampire.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vampire.hardware.Arm;
import org.firstinspires.ftc.teamcode.vampire.hardware.Intake;
import org.firstinspires.ftc.teamcode.vampire.roadrunner.drive.VampireRRDrive;

@Autonomous(name="Vampire: RRDWa", group="Vampire")
public class BLWa extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

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
        if(isStopRequested()) return;

        // Deploy cargo
        drive.followTrajectory(deploy);
        arm.setLift(3);
        sleep(1000);
        intake.reverse();
        sleep(2000);
        intake.stop();

        // Go to warehouse
        drive.followTrajectory(warehouse);

    }

}
