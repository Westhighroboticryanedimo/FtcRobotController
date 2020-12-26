package org.firstinspires.ftc.teamcode.hardware.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;

@Config
public class DriveConstants {

    public static final double TICKS_PER_REV = 1680;
    public static final double MAX_RPM = 105;

    public static double WHEEL_RADIUS = 4.8; // In centimeters
    public static double GEAR_RATIO = 1; // Output (wheel) speed / Input (motor) speed
    public static double TRACK_WIDTH = 20; // Distance between the two wheels in centimeters

    public static double kV = 0.98 / rpmToVelocity(MAX_RPM);
    public static double kA = 0.00004;
    public static double kStatic = 0.08722;

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            30.0, 30.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF() {
        return 32767 * 60.0 / (MAX_RPM * TICKS_PER_REV);
    }

}