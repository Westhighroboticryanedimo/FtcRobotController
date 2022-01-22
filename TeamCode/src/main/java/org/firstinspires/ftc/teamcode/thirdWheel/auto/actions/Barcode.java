/*package org.firstinspires.ftc.teamcode.thirdWheel.auto.actions;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.DriveThirdWheel;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Lift;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Barcode extends BaseHardware {

    private DriveThirdWheel drive;
    private Lift lift;
    private DistanceSensor distanceSensor;
    private final int PIECE_DIST = 6;
    private int level = 0;

    public Barcode(DriveThirdWheel d, Lift l, DistanceSensor ds) {
        init(d, l, ds);
    }

    public void init(DriveThirdWheel d, Lift l, DistanceSensor ds) {
        drive = d;
        lift = l;
        distanceSensor = ds;
    }

    public void detect() {
        intake.inhale();
        while (distanceSensor.getDistance(DistanceUnit.INCH) > 8) {
            drive.setPowers(0.25, -0.25, 0.25, -0.25);
        }
        drive.stop();
        drive.moveUntil(0.25, 0, lift.check());


}
 */