package org.firstinspires.ftc.teamcode.thirdWheel.auto.actions;

import com.qualcomm.ftccommon.configuration.EditLegacyServoControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Park extends BaseHardware {

    private DriveThirdWheel drive;
    private Lift lift;

    public Park(LinearOpMode opMode, DriveThirdWheel d, Lift l) {
        super(opMode);
        init(d, l);
    }

    public void init(DriveThirdWheel d, Lift l) {
        drive = d;
        lift = l;
    }

    // Assumes in line with center of alliance shipping hub
    public void warehouse(int color) {
        lift.override(2, -1);
        lift.assist();
        while (!lift.arrived()) {
            lift.assist();
        }
        if (color == 1) {
            // drive.frrNormie(2.5, 0, -1, 0.5, 1, -1, 0, 0, 0);
            drive.frrNormie(2.5, 0, -1, 0, 0, 0, 0, 0, 0);
            // drive.frrNormieByTime(2.5, 2.5, -1, 0.5, 1, 0, 0);
        } else if (color == 2) {
            drive.frrNormie(2.5, 0, -1, 0.5, 1, -1, 0, 0, 0);
        }
        // drive.frrNormie(0, 0, 0, 1, 0, 1, 0, 0, 0);
        lift.override(0, -1);
        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < 3)  {
            lift.assist();
        }
    }

}
