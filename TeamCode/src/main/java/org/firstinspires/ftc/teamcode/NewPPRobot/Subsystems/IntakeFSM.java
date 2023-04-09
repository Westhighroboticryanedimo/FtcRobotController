package org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeFSM {
    private ServoEx clawServo;
    private ServoEx wristServo;
    private ServoEx pivotServo1;
    private ServoEx pivotServo2;

    int intakeState = 1;
    int intakeFSMRunning = 0;

    public ElapsedTime timer = new ElapsedTime();

    public void intakeInit(HardwareMap hardwareMap) {
        clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 360);
        wristServo = new SimpleServo(hardwareMap, "wristServo", 0, 360);
        pivotServo1 = new SimpleServo(hardwareMap, "pivotServo1", 0, 360);
        pivotServo2 = new SimpleServo(hardwareMap, "pivotServo2", 0, 360);
    }

    public void intake() {
        if (intakeFSMRunning == 1) {
            switch (intakeState) {
                case 1:
                    //Close Claw 200 ms
                    clawServo.turnToAngle(200);
                    if (timer.milliseconds() > 300) {
                        timer.reset();
                        intakeState = 2;
                    }
                    break;
                case 2:
                    //Pivot Forward 500 ms
                    pivotServo1.turnToAngle(290);
                    pivotServo2.turnToAngle(70);
                    if (timer.milliseconds() > 500) {
                        timer.reset();
                        intakeState = 3;
                    }
                    break;
                case 3:
                    //Wrist Pivot 500 ms
                    wristServo.turnToAngle(16);
                    if (timer.milliseconds() > 800) {
                        timer.reset();
                        intakeState = 4;
                    }
                    break;
                case 4:
                    //Pivot to driving position
                    pivotServo1.turnToAngle(260);
                    pivotServo2.turnToAngle(100);
                    intakeFSMRunning = 0;
                    intakeState = 1;
                    break;
            }
        }
    }

    public void resetTimer() {
        timer.reset();
    }

    public void startIntakeFSM() {
        intakeFSMRunning = 1;
    }

}
