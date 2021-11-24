package org.firstinspires.ftc.teamcode.jeff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;
import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

public class JeffDrive extends BaseHardware {

    //public ColorSensor color;
    public Servo highFive;
    public Servo wave;
    public ElapsedTime fiveTimer;
    public ElapsedTime waveTimer;
    public boolean isWaving = false;
    public boolean isFiving = false;
    //public int blueCount = 0;
    //public int redCount = 0;

    // For teleop
    public JeffDrive(OpMode opMode, HardwareMap hwMap) {

        //super(opMode, hwMap);
        super(opMode);
        setup(hwMap);

    }

    // Setup
    private void setup(HardwareMap hwMap) {
/*
        // PID Values
        setPidDrive(0.02, 0, 0);
        setPidSpeed(0.03, 0.002, 0);
        setPidTurn(0.03, 0.001, 0);

        // Robot characteristics
        setWheelDiameter(3.9);
        setTicksPerRev(537.6);
*/
        fiveTimer = new ElapsedTime();
        waveTimer = new ElapsedTime();

        // Color sensor on bot
        //color = hwMap.get(ColorSensor.class, "color");

        highFive = hwMap.get(Servo.class, "highFive");
        wave = hwMap.get(Servo.class, "wave");

    }
/*
    public void checkColor() {

        if (color.blue() > 5000) {

            blueCount++;
            if (blueCount == 1) wave(true);

        } else blueCount = 0;

        if (color.red() > 4000) {

            redCount++;
            if (redCount == 1) highFive(true);

        } else redCount = 0;

    }
*/
    public void wave(boolean button) {

        if (!isWaving) {

            wave.setPosition(0.3);
            waveTimer.reset();
            if (button) {

                wave.setPosition(0.7);
                isWaving = true;

            }

        }
        if (waveTimer.milliseconds() > 500) isWaving = false;

    }

    public void highFive(boolean button) {

        if (!isFiving) {

            highFive.setPosition(0.8);
            fiveTimer.reset();
            if (button) {

                highFive.setPosition(0.5);
                isFiving = true;

            }

        }
        if (fiveTimer.milliseconds() > 600) isFiving = false;

    }

}