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
    public Boolean helpme = false;

    public ElapsedTime timer = new ElapsedTime();

    public Place(HardwareMap hwMap, String lm, String rm, String fl, String fr, String g) {
        drcb = new DRCB(hwMap, lm, rm);
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
        level = l;
        gripper.setLevel(level);
        timer.reset();
    }

    public void dropAndLower() {
        state = State.LOWERING;
        gripper.open();
//        timer.reset();
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
                    drcb.setLevel(level);
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
}
