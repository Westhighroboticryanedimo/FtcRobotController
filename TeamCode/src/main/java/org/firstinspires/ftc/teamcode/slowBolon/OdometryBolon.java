package org.firstinspires.ftc.teamcode.slowBolon;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;
import com.qualcomm.robotcore.util.SerialNumber;

public class OdometryBolon implements Runnable{
    private DcMotor wheel;
    public int distance;
    private int lastcheck;

    public void init(DcMotor in) {
        wheel = in;
    }

    public void resetdistance() {distance = 0;}

    public void updatedistance() {
        int distancechange = wheel.getCurrentPosition()-lastcheck;
        distance += distancechange;
        lastcheck = wheel.getCurrentPosition();
    }

    @Override
    public void run() {
        updatedistance();
    }
}
