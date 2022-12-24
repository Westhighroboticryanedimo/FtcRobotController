package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.util.control.Control;

public class DRCB {
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public TouchSensor touch;

    public int level = 0;
    public int oldLevel = 0;

    private static final double LEVELS[] = {0, 170, 254, 345};
    private static final double LIFT_POWER = 1.0;

    // TODO: find the right values for this
    private static final double TICKS_PER_REV = 1120;
    private static final double L_0 = 4.2; // motor to pivot dist
    private static final double L_A = 1.5; // bottom linkage
    private static final double L_B = 4.5; // top linkage
    private static final double L_OFFSET = 1.9; // linkage attachment dist
    private static final double THETA_0 = 2; // angle between horizontal and L_0 in rad
    private static final double kTau_ff = 0.16; // gain for torque feedforward

    public double p = 0.02;
    public double i = 0.0;
    public double d = 0.002;
    private PIDController pid = new PIDController(p, i, d);

    public double ff = 0.0;
    public double output = 0.0;
    public double total = 0.0;

    public double setpoint = 0.0;
    public ElapsedTime timer = new ElapsedTime();

    public Boolean useMotionProfile = true;
    public Boolean justFeedforward = false;

    public DRCB(HardwareMap hwMap, String lm, String rm, String ts) {
        pid.reset();
        pid.setInputRange(0, LEVELS[3]);
        pid.setOutputRange(0, LIFT_POWER);
        pid.setTolerance(2);
        pid.enable();

        leftMotor = hwMap.get(DcMotor.class, lm);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor = hwMap.get(DcMotor.class, rm);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setPower(0);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        touch = hwMap.get(TouchSensor.class, ts);
    }

    public void setLevel(int l) {
        oldLevel = level;
        level = l;
//        pid.setSetpoint(LEVELS[level]);
        timer.reset();
    }

    public void run() {
        if (touch.isPressed()) {
            reset();
        }
        // TODO: make this faster
        setpoint = Control.trapMotion(1000.0, 600.0, LEVELS[oldLevel], LEVELS[level], timer.seconds());
        if (useMotionProfile) {
            pid.setSetpoint(setpoint);
        }
        if (touch.isPressed()) {
            reset();
        }
        // TODO: find feedforward offset value
        // angle of motor between lift rest and lift horizontal
        ff = calculateFeedforward(getPosition() - 130);
        output = pid.performPID(getPosition());
        total = ff + output;
        if (justFeedforward) {
            total = ff;
        }
        // if going down, reduce output cause gravity
        // TODO: take care of this in the model
        // TODO: change this to scale
        if (total < 0) {
            total = 0.01;
        }
        leftMotor.setPower(total);
        rightMotor.setPower(total);
    }

    public double getPosition() {
        return rightMotor.getCurrentPosition();
    }

    private void reset() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updatePID() {
        pid.setPID(p, i, d);
    }

    public double calculateFeedforward(double ticks) {
        double angle = 2*Math.PI*ticks/TICKS_PER_REV;
        double l_1 = Math.sqrt(Math.pow(L_A, 2)
                               + Math.pow(L_0, 2)
                               - 2*L_A*L_0*Math.cos(THETA_0 - angle));
        double theta_ab = lawOfCos(L_0, l_1, L_A) + lawOfCos(L_OFFSET, l_1, L_B);
        double theta_bc = lawOfCos(l_1, L_B, L_OFFSET);
        double theta = Math.PI - (theta_ab - angle) - theta_bc;

        return kTau_ff*Math.cos(theta)/(Math.sin(theta_ab)*Math.sin(theta_bc));
    }

    // calculate angle of an SSS triangle
    // @param a is the side opposite to the angle you want to find
    private double lawOfCos(double a, double b, double c) {
        return Math.acos((Math.pow(b, 2) + Math.pow(c, 2) - Math.pow(a, 2))
                         / (2*b*c));
    }

    public Boolean arrived() {
        return pid.onTarget();
    }

    public int getCurrentLeftTicks() {
        return leftMotor.getCurrentPosition();
    }

    public int getCurrentRightTicks() {
        return rightMotor.getCurrentPosition();
    }
}
