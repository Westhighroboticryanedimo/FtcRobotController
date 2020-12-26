package org.firstinspires.ftc.teamcode.uncledrew.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {

    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    public Sensors(HardwareMap hwMap) {

        colorSensor = hwMap.get(ColorSensor.class, "color");
        distanceSensor = hwMap.get(DistanceSensor.class, "color");

    }

    public double getDistanceCM() {

        return distanceSensor.getDistance(DistanceUnit.CM);

    }

    public int[] getRGB() {

        return new int[]{colorSensor.red(), colorSensor.green(), colorSensor.blue()};

    }

    public boolean isSkystone() {

        if (getDistanceCM() < 8 && getDistanceCM() >= 7)
            return getRGB()[0] < 200;
        else if (getDistanceCM() < 7 && getDistanceCM() >= 6)
            return getRGB()[0] < 250;
        else if (getDistanceCM() < 6 && getDistanceCM() >= 4)
            return getRGB()[0] < 375;
        else if (getDistanceCM() < 4 && getDistanceCM() >= 3)
            return getRGB()[0] < 750;
        else if (getDistanceCM() < 3 && getDistanceCM() >= 2)
            return getRGB()[0] < 1000;
        else if (getDistanceCM() < 2 && getDistanceCM() >= 1)
            return getRGB()[0] < 3000;
        else if (getDistanceCM() < 1)
            return getRGB()[0] < 5000;
        else
            return false;

    }

}
