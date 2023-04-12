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
        LOWERING,
        STACK_UP,
        DIP_DOWN,
        DIP_UP,
        WAVE
    } public State state = State.INTAKING;

    public DRCB drcb;
    public Gripper gripper;

    private int level = 0;
    private int oldLevel = 0;
    public Boolean helpme = false;
    private Boolean didIAlreadyCallSetLevelOrNot = false;

    public ElapsedTime timer = new ElapsedTime();

    private int stackCount = 0;

    public Place(HardwareMap hwMap, String lm, String rm, String ts, String fl, String fr, String g, Gripper.Alliance alliance) {
        drcb = new DRCB(hwMap, lm, rm, ts);
        gripper = new Gripper(hwMap, fl, fr, g, alliance);
    }

    public void intake() {
        state = State.INTAKING;
        level = 0;
        gripper.setLevel(level);
        timer.reset();
    }

    public void pickup() {
        state = State.WAITING_FOR_CLOSE;
        gripper.close();
        timer.reset();
    }

    public void raise(int l) {
        state = State.RAISING;
        level = l;
        drcb.setLevel(level);
        timer.reset();
    }

    public void dropAndLower() {
        state = State.LOWERING;
        level = 0;
        gripper.open();
        didIAlreadyCallSetLevelOrNot = false;
        timer.reset();
    }

    public void goToStack() {
        if (stackCount > 3) {
            return;
        }
        state = State.RAISING;
        level = 4 + stackCount;
        stackCount++;
        drcb.setLevel(level);
        timer.reset();
    }

    // for when you make an oopsie
    public void decrementStack() {
        stackCount--;
    }

    public void liftOffStack() {
        state = State.STACK_UP;
        level = 1;
        gripper.close();
        timer.reset();
    }

    public void dip(boolean input) {
        if (input) {
            state = State.DIP_DOWN;
        } else {
            state = State.DIP_UP;
        }
        timer.reset();
    }

    public void wave() {
        state = State.WAVE;
        level = 3;
        drcb.setLevel(level);
        gripper.close();
        timer.reset();
    }

    public void run(double input) {
        switch(state) {
            case INTAKING:
                if (timer.milliseconds() > 50) {
                    gripper.open();
                    timer.reset();
                }
                if (gripper.in()) {
                    pickup();
                }
                break;
            case WAITING_FOR_CLOSE:
                if (timer.milliseconds() > 300) {
                    helpme = true;
                    gripper.setLevel(-1);
                    timer.reset();
                    state = State.PICKED;
                }
                break;
            case PICKED:
                break;
            case RAISING:
                int waitTime = 300+(50*Math.abs(oldLevel-level));
                if (level >= 4) {
                    waitTime = 150;
                }
                if (timer.milliseconds() > waitTime) {
                    gripper.setLevel(level);
                    timer.reset();
                }
                break;
            // TODO: watch out for catching of the bottom of claw
            case LOWERING:
                if (timer.milliseconds() > 150) {
//                    gripper.close();
                }
                if (timer.milliseconds() > 300) {
                    gripper.setLevel(-1);
                    if (!didIAlreadyCallSetLevelOrNot) {
                        drcb.setLevel(level);
                        didIAlreadyCallSetLevelOrNot = true;
                    }
                }
                break;
            case STACK_UP:
                if (timer.milliseconds() > 300) {
                    drcb.setLevel(level);
                    timer.reset();
                }
                break;
            case DIP_DOWN:
                gripper.dipDown();
                timer.reset();
                break;
            case DIP_UP:
                gripper.dipUp();
                timer.reset();
                break;
            case WAVE:
                if (timer.milliseconds() > 750) {
                    gripper.open();
                } else if (timer.milliseconds() > 1000) {
                    gripper.close();
                } else if (timer.milliseconds() > 1250) {
                    gripper.open();
                } else if (timer.milliseconds() > 1500) {
                    gripper.close();
                } else if (timer.milliseconds() > 1750) {
                    level = 0;
                    drcb.setLevel(0);
                }
                break;
        }
        drcb.run(input);
    }

    public int getLeftPos() {
        return drcb.getCurrentLeftTicks();
    }
    public int getRightPos() {
        return drcb.getCurrentRightTicks();
    }
}
