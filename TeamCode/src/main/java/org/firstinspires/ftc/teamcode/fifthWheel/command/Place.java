package org.firstinspires.ftc.teamcode.fifthWheel.command;

import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.DRCB;
import org.firstinspires.ftc.teamcode.fifthWheel.subsystem.Gripper;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Place {

    enum State {
        INTAKING,
        PICKED,
        WAITING_FOR_CLOSE,
        RAISING,
        LOWERING
    } public State state = State.INTAKING;

    public DRCB drcb;
    private Gripper gripper;

    private int level = 0;
    private int oldLevel = 0;
    public Boolean helpme = false;

    public ElapsedTime timer = new ElapsedTime();

    public Place(HardwareMap hwMap, String lm, String rm, String ts, String fl, String fr, String g) {
        drcb = new DRCB(hwMap, lm, rm, ts);
        gripper = new Gripper(hwMap, fl, fr, g);
    }

    public void intake() {
        state = State.INTAKING;
        level = 0;
        gripper.setLevel(level);
        gripper.open();
        drcb.setLevel(level);
    }

    public void pickup() {
        state = State.WAITING_FOR_CLOSE;
        gripper.close();
        timer.reset();
    }

    public void raise(int l) {
        state = State.RAISING;
        oldLevel = level;
        level = l;
        // if raising, raise lift then move wrist
        // if lowering, move wrist then lower lift
        if (oldLevel < level) {
            drcb.setLevel(level);
        } else {
            gripper.setLevel(level);
        }
        timer.reset();
    }

    public void dropAndLower() {
        state = State.LOWERING;
        gripper.open();
        timer.reset();
    }

    public void run() {
        switch(state) {
            case INTAKING:
                break;
            case WAITING_FOR_CLOSE:
                if (timer.milliseconds() > 400) {
                    helpme = true;
                    gripper.setLevel(-1);
                    timer.reset();
                    state = State.PICKED;
                }
                break;
            case PICKED:
                break;
            case RAISING:
                if (timer.milliseconds() > 300) {
                    if (oldLevel < level) {
                        gripper.setLevel(level);
                    } else {
                        drcb.setLevel(level);
                    }
                    timer.reset();
                }
                break;
            case LOWERING:
                if (timer.milliseconds() > 400) {
                    gripper.setLevel(-1);
                }
//                if (timer.milliseconds() > 500 ) {
//                    level = 0;
//                    drcb.setLevel(level);
//                }
//                if (drcb.arrived()) {
//                    state = State.INTAKING;
//                }
                break;
        }
        drcb.run();
    }

    public int getLeftPos() {
        return drcb.getCurrentLeftTicks();
    }
    public int getRightPos() {
        return drcb.getCurrentRightTicks();
    }
}
