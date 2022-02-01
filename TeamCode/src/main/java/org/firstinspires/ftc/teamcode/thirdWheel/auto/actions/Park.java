package org.firstinspires.ftc.teamcode.thirdWheel.auto.actions;

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

    public void comeBackPlease(int color) {
        drive.move(0.25, 4, 180);
        if (color == 1) {
            drive.move(0.25, 40, -90);
        } if (color == 2) {
            drive.move(0.25, 20, 90);
        }
    }

    // Assumes in line with center of alliance shipping hub
    public void warehouse(int color) {
        lift.override(2, -1);
        lift.assist();
        while (!lift.arrived()) {
            lift.assist();
        }
        if (color == 1) {
            drive.move(0.5, 80, 265);
            drive.move(0.5, 15, 0);
        } else if (color == 2) {
            drive.move(0.5, 80, 95);
            drive.move(0.5, 15, 0);
        }
        lift.override(0, -1);
        //elapsedtime runtime = new elapsedtime();
        /*while (runtime.seconds() < 3)  {
            lift.assist();
        }*/
    }

}
