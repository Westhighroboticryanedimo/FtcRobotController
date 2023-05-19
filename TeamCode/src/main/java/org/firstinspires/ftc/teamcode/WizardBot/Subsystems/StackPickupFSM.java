package org.firstinspires.ftc.teamcode.WizardBot.Subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class StackPickupFSM {
    private ServoEx clawServo;
    private ServoEx wristServo;
    private ServoEx pivotServo1;
    private ServoEx pivotServo2;

    int state = 1;
    int stackPickupFSMRunning = 0;

    public ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {
        clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 360);
        wristServo = new SimpleServo(hardwareMap, "wristServo", 0, 360);
        pivotServo1 = new SimpleServo(hardwareMap, "pivotServo1", 0, 360);
        pivotServo2 = new SimpleServo(hardwareMap, "pivotServo2", 0, 360);
    }

    public void pickup() {
        if (stackPickupFSMRunning == 1) {
            switch (state) {
                case 1:
                    //Close Claw 100 ms
                    clawServo.turnToAngle(135);
                    if (timer.milliseconds() > 100) {
                        timer.reset();
                        state = 2;
                    }
                    break;
                case 2:
                    //Pivot to Driving
                    pivotServo1.turnToAngle(260);
                    pivotServo2.turnToAngle(100);
                    stackPickupFSMRunning = 0;
                    state = 1;
                    break;
            }
        }
    }

    public void resetTimer() {
        timer.reset();
    }

    public void startStackPickupFSM() {
        stackPickupFSMRunning = 1;
    }

}
