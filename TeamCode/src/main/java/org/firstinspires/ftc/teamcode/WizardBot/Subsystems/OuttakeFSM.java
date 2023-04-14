package org.firstinspires.ftc.teamcode.WizardBot.Subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OuttakeFSM {
    private ServoEx clawServo;
    private ServoEx wristServo;
    private ServoEx pivotServo1;
    private ServoEx pivotServo2;

    int outtakeState = 1;
    int outtakeFSMRunning = 0;

    public ElapsedTime timer = new ElapsedTime();

    public void outtakeInit(HardwareMap hardwareMap) {
        clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 360);
        wristServo = new SimpleServo(hardwareMap, "wristServo", 0, 360);
        pivotServo1 = new SimpleServo(hardwareMap, "pivotServo1", 0, 360);
        pivotServo2 = new SimpleServo(hardwareMap, "pivotServo2", 0, 360);
    }

    public void outtake() {
        if (outtakeFSMRunning == 1) {
            switch (outtakeState) {
                case 1:
                    //Pivot to flat 700 ms
                    pivotServo1.turnToAngle(360);
                    pivotServo2.turnToAngle(0);
                    if (timer.milliseconds() > 1000) {
                        timer.reset();
                        outtakeState = 2;
                    }
                    break;
                case 2:
                    //Open Claw 50 ms
                    clawServo.turnToAngle(155);
                    if (timer.milliseconds() > 50) {
                        timer.reset();
                        outtakeState = 3;
                    }
                    break;
                case 3:
                    //Pivot 200 ms
                    pivotServo1.turnToAngle(120);
                    pivotServo2.turnToAngle(240);
                    if (timer.milliseconds() > 150) {
                        timer.reset();
                        outtakeState = 4;
                    }
                    break;
                case 4:
                    // Close Claw and Pivot Wrist
                    clawServo.turnToAngle(190);
                    wristServo.turnToAngle(256);
                    outtakeFSMRunning = 0;
                    outtakeState = 1;
                    break;
            }
        }
    }

    public void resetTimer() {
        timer.reset();
    }

    public void startOuttakeFSM() {
        outtakeFSMRunning = 1;
    }

}
