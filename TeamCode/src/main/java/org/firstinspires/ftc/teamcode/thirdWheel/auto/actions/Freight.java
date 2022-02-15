/*package org.firstinspires.ftc.teamcode.thirdWheel.auto.actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Freight extends BaseHardware {

    private DriveThirdWheel drive;
    private Lift lift;

    public Freight(LinearOpMode opMode, DriveThirdWheel d, Lift l) {
        super(opMode);
        init(d, l);
    }

    public void init(DriveThirdWheel d, Lift l) {
        drive = d;
        lift = l;
    }

    public void enter(int color) {
        lift.override(2, -1);
        lift.assist();
        while (!lift.arrived()) {
            lift.assist();
        }
        if (color == 1) {
            drive.frrMove(2.25, 2.25, -1, 1, -1, 0, 0);
        } else if (color == 2) {
            drive.frrMove(2.25, 2.25, 1, 1, -1, 0, 0);
        }
    }
}
*/