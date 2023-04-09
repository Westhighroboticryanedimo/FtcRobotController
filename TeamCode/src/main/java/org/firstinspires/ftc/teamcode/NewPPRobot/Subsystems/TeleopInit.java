package org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TeleopInit {
    private ServoEx clawServo;
    private ServoEx wristServo;
    private ServoEx pivotServo1;
    private ServoEx pivotServo2;

    int state = 1;
    int FSMRunning = 0;

    public ElapsedTime timer = new ElapsedTime();

    public void teleopInit(HardwareMap hardwareMap) {
        clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 360);
        wristServo = new SimpleServo(hardwareMap, "wristServo", 0, 360);
        pivotServo1 = new SimpleServo(hardwareMap, "pivotServo1", 0, 360);
        pivotServo2 = new SimpleServo(hardwareMap, "pivotServo2", 0, 360);
    }

    public void init() {
        if (FSMRunning == 1) {
            switch (state) {
                case 1:
                    //Pivot Forward 800 ms
                    pivotServo1.turnToAngle(290);
                    pivotServo2.turnToAngle(70);
                    if (timer.milliseconds() > 500) {
                        timer.reset();
                        state = 2;
                    }
                    break;
                case 2:
                    //Wrist Pivot 500 ms
                    wristServo.turnToAngle(256);
                    clawServo.turnToAngle(155);
                    if (timer.milliseconds() > 800) {
                        timer.reset();
                        state = 3;
                    }
                    break;
                case 3:
                    //Pivot to intake position
                    pivotServo1.turnToAngle(115);
                    pivotServo2.turnToAngle(240);
                    FSMRunning = 0;
                    state = 1;
                    break;
            }
        }
    }

    public void resetTimer() {
        timer.reset();
    }

    public void startTeleopInit() {
        FSMRunning = 1;
    }

}
