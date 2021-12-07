package org.firstinspires.ftc.teamcode.hardware.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
/*
public class DcMotorUtils {
    // Move the motor to the position specified, in ticks
    // Except it only sets the power and you have to run it in the loop() thing, because yes
    static public void moveByTicks(DcMotor motor, double desiredTicks, double startDec, double tolerance) {
        // Deceleration: min(x * 1/startDec, 1) where x = difference
        if (Math.abs(getDifference(motor, desiredTicks)) > tolerance ) {
            int direction = (int) (getDifference(motor, desiredTicks)/Math.abs(getDifference(motor, desiredTicks)));
            motor.setPower(Math.min(Math.abs(getDifference(motor, desiredTicks))*(1.0/startDec), 1) * direction);
        } else {
            motor.setPower(0);
        }
    }

    static public void moveByDistance(DcMotor motor, double distance, double ticksPerRev, double circumference, double startDec, int tolerance) {
        moveByTicks(motor, ((distance/circumference) * ticksPerRev), startDec, tolerance);
    }

    static public double getDifference(DcMotor motor, double desiredTicks) { return desiredTicks - getCurrentTicks(motor); }
    static public int getCurrentTicks(DcMotor motor) { return motor.getCurrentPosition(); }
}
*/