package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import java.lang.Math;
// import java.lang.Thread;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.control.DcMotorUtils;
import org.firstinspires.ftc.teamcode.PIDController;

public class DRCB {
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    private int level = 0;
    private int ticks = 0;

    private static final double LEVELS[] = {0, 80, 100, 160, 320};
    private static final double LIFT_POWER = 0.57;
    private static final double LOWER_POWER = -0.05;

    private static final double TICKS_PER_REV = 1120;
    private static final double L_0 = 3; // motor to pivot dist
    private static final double L_A = 2; // bottom linkage
    private static final double L_B = 3.75; // top linkage
    private static final double L_OFFSET = 2.4; // linkage attachment dist
    private static final double THETA_0 = 114; // angle between horizontal and L_0
    private static final double kP_ff = 0.01; // gain for torque feedforward

    private PIDController pidArm = new PIDController(1, 0, 0);

    public DRCB(HardwareMap hwMap) {
        init(hwMap);
    }

    private void init(HardwareMap hwMap) {
        pidArm.reset();
        pidArm.setInputRange(0, LEVELS[4]);
        pidArm.setOutputRange(0, LIFT_POWER);
        pidArm.setTolerance(10);

        leftMotor = hwMap.get(DcMotor.class, "leftMotor");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotor = hwMap.get(DcMotor.class, "rightMotor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setPower(0);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double calculateFeedforward(double angle) {
        double l_1 = Math.sqrt(Math.pow(L_A, 2)
                               + Math.pow(L_0, 2)
                               - 2*L_A*L_0*Math.cos(THETA_0 - angle));
        double theta_ab = lawOfCos(L_0, l_1, L_A) + lawOfCos(L_OFFSET, l_1, L_B);
        double theta_bc = lawOfCos(l_1, L_B, L_OFFSET);
        double theta = 180 - (theta_ab - angle) - theta_bc;

        return kP_ff*Math.cos(theta)/(Math.sin(theta_ab)*Math.sin(theta_bc));
    }

    // calculate angle of an SSS triangle
    // @param a is the side opposite to the angle you want to find
    private double lawOfCos(double a, double b, double c) {
        return Math.acos((Math.pow(b, 2) + Math.pow(c, 2) - Math.pow(a, 2))
                         / (2*b*c));
    }

    public int getCurrentLeftTicks() {
        return leftMotor.getCurrentPosition();
    }

    public int getCurrentRightTicks() {
        return rightMotor.getCurrentPosition();
    }
}
