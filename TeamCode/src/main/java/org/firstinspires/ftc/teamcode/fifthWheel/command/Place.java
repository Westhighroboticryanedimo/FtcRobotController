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

    public Place(HardwareMap hwMap, String lm, String rm, String ts, String fl, String fr, String gl, String gr) {
        drcb = new DRCB(hwMap, lm, rm, ts);
        gripper = new Gripper(hwMap, fl, fr, gl, gr);
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
        timer.reset();
    }

    public void run() {
        switch(state) {
            case INTAKING:
                if (timer.milliseconds() > 275) {
                    gripper.open();
                    timer.reset();
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
                if (timer.milliseconds() > 300) {
                    gripper.setLevel(level);
                    timer.reset();
                }
                break;
            case LOWERING:
                if (timer.milliseconds() > 200) {
                    gripper.close();
                }
                if (timer.milliseconds() > 400) {
                    gripper.setLevel(-1);
                    drcb.setLevel(level);
                }
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
