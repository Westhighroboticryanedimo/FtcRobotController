package org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.LinearSlide;
import org.firstinspires.ftc.teamcode.thirdWheel.hardware.lift.Intake;

public class Lift extends BaseHardware {
    private LinearSlide linearSlide;
    private Intake intake;
    private TouchSensor touchSensor;
    private int currentLevel = 0;
    private int currentIntakeState = 2;
    private enum State {
        INHALING,
        CAPTURED,
        OVERRIDDEN
    }

    private State state = State.INHALING;
    
    public Lift(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode);
        init(hwMap);
    }

    public Lift(OpMode opMode, HardwareMap hwMap) {
        super(opMode);
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        linearSlide =  new LinearSlide(hwMap);
        intake = new Intake(hwMap);
        touchSensor = hwMap.get(TouchSensor.class, "touchSensor");
    }

    // WARNING: Needs to be called in a loop
    public void assist() {
        switch (state) {
        case INHALING:
            // intake.in();
            linearSlide.setLevel(0);
            if (touchSensor.isPressed()) {
                state = State.CAPTURED;
            }
            break;
        case CAPTURED:
            intake.stop();
            linearSlide.setLevel(1);
            break;
        case OVERRIDDEN:
            intake.setState(currentIntakeState);
            linearSlide.setLevel(currentLevel);
            break;
        }
    }

    public void override(int level, int intakeState) {
        state = State.OVERRIDDEN;
        currentLevel = level;
        currentIntakeState = intakeState;
    }

    public void inhale() {
        state = State.INHALING;
    }

    public int getLevel() { return linearSlide.getLevel(); }
    public double getCurrentTicks() { return linearSlide.getCurrentTicks(); }
    public boolean touchPressed() { return touchSensor.isPressed(); }
}
