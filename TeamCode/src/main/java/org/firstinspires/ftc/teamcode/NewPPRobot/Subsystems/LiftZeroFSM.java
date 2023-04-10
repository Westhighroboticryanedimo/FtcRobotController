package org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

public class LiftZeroFSM {

    int state = 1;
    int FSMRunning = 0;
    int liftResting = 1;
    int FSMLowering = 0;

    private DcMotor lift1;
    private DcMotor lift2;
    private TouchSensor liftLimit;
    private ChodeLift lift;
    private ServoEx clawServo;

    public ElapsedTime timer = new ElapsedTime();

    public void liftZeroInit(HardwareMap hardwareMap, ChodeLift init_lift) {
        lift = init_lift;
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        liftLimit = hardwareMap.get(TouchSensor.class, "liftLimit");
        clawServo = new SimpleServo(hardwareMap, "clawServo", 0, 360);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.liftInit(hardwareMap);
    }

    public void liftZero() {
        if (FSMRunning == 1) {
            switch (state) {
                case 1:
                    lift.setLiftPos(150);
                    FSMRunning = 1;
                    if (lift.arrived() == 1) {
                        state = 2;
                        timer.reset();
                    }
                    break;
                case 2:
                    if (timer.milliseconds() > 500) {
                        state = 3;
                        liftResting = 1;
                        clawServo.turnToAngle(155);
                    }
                case 3:
                    FSMLowering = 1;
                    lift1.setPower(-0.1);
                    lift2.setPower(0.1);
                    if (liftLimit.isPressed() == true) {
                        lift1.setPower(0);
                        lift2.setPower(0);
                        FSMLowering = 0;
                        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        FSMRunning = 0;
                        state = 1;
                    }
                    break;
            }
        }
    }

    public void startLiftZeroFSM() {
        FSMRunning = 1;
    }

    public void setLiftResting(int newValue) {
        liftResting = newValue;
    }

    public int getLiftResting() {
        return(liftResting);
    }

    public int getFSMLowering() {return(FSMLowering);}

}
