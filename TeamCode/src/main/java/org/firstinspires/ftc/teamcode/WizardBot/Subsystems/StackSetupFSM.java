package org.firstinspires.ftc.teamcode.WizardBot.Subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class StackSetupFSM {
    private ServoEx clawServo;
    private ServoEx wristServo;
    private ServoEx pivotServo1;
    private ServoEx pivotServo2;

    int state = 1;
    int stackSetupFSMRunning = 0;

    public ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {
        clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 360);
        wristServo = new SimpleServo(hardwareMap, "wristServo", 0, 360);
        pivotServo1 = new SimpleServo(hardwareMap, "pivotServo1", 0, 360);
        pivotServo2 = new SimpleServo(hardwareMap, "pivotServo2", 0, 360);
    }

    public void setup() {
        if (stackSetupFSMRunning == 1) {
            switch (state) {
                case 1:
                    //Open Claw and Pivot Forward 500 ms
                    clawServo.turnToAngle(155);
                    pivotServo1.turnToAngle(360);
                    pivotServo2.turnToAngle(0);
                    if (timer.milliseconds() > 500) {
                        timer.reset();
                        state = 2;
                    }
                    break;
                case 2:
                    //Wrist Twist
                    wristServo.turnToAngle(16);
                    stackSetupFSMRunning = 0;
                    state = 1;
                    break;
            }
        }
    }

    public void resetTimer() {
        timer.reset();
    }

    public void startStackSetupFSM() {
        stackSetupFSMRunning = 1;
    }

}
