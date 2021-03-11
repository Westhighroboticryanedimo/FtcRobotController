package org.firstinspires.ftc.teamcode.marinara.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;
import org.opencv.core.Mat;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class Shooter extends BaseHardware {

    // Unit conversions
    private static final double R_TICKS_PER_REV = 103.6;
    private static final double L_TICKS_PER_REV = 74.3; // Empirically saw that the tpr for left motor is 74.3
    private static final double R_REV_PER_TICKS = 1 / R_TICKS_PER_REV;
    private static final double L_REV_PER_TICKS = 1 / L_TICKS_PER_REV;

    // Final variables that won't change; used for calculations
    private static final double GEAR_RATIO = 80.0 / 32.0;
    private static final double WHEEL_DIAMETER_IN = 4;
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_IN * METERS_PER_INCHES * Math.PI;

    // Motors
    private DcMotor shooterL = null;
    private DcMotor shooterR = null;


    // Servo
    private Servo stopper = null;
    private static final double STOP_POS = 0.45;
    private static final double OPEN_POS = 0;

    // Timer
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    // Variables to use when calculating speed
    private double lastTime = 0;
    private long lastPosL = 0;
    private long lastPosR = 0;
    private static final double CALC_TIME_INTERVAL = 0.001;
    private static final double AUTO_SHOOT_TIME = 6;

    // Misc variables
    private static final double SHOOT_WAIT = 2;
    private static final double SHOOT_POW_L = 1;
    private static final double SHOOT_POW_R = 0;

    // Teleop constructor
    public Shooter(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    // Autonomous constructor
    public Shooter(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setup(hwMap);

    }

    private void setup(HardwareMap hwMap) {

        // Set up motors

        shooterL = hwMap.get(DcMotor.class, "shooterL");
        shooterR = hwMap.get(DcMotor.class, "shooterR");

        shooterL.setDirection(DcMotor.Direction.FORWARD);
        shooterR.setDirection(DcMotor.Direction.REVERSE);

        shooterL.setPower(0);
        shooterR.setPower(0);

        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set up servos
        stopper = hwMap.get(Servo.class, "stopper");
        stopper.setPosition(STOP_POS);

    }

    public void toggleStopper(boolean button) {

        if (button) {

            if (stopper.getPosition() == STOP_POS) {

                stopper.setPosition(OPEN_POS);

            } else {

                stopper.setPosition(STOP_POS);

            }

        }

    }

    private double calculateMotorSpeed(double power, double voltage) {

        // Linear proportionality between voltage and power
        final double POW_PER_VOLT = 0.1;
        double voltDiff = 13 - voltage;
        double powDiff = voltDiff * POW_PER_VOLT;
        return power + powDiff;

    }

    public void shoot(double powerL, double powerR, double voltage, Intake intake) {

        runtime.reset();
        lastTime = timer.seconds();
        while (linearOpMode.opModeIsActive() && runtime.seconds() < AUTO_SHOOT_TIME) {

            // Open shooter
            stopper.setPosition(OPEN_POS);

            // Difference in time since last time
            double timeDiff = timer.seconds() - lastTime;

            // Calculate motor speeds
            powerL = calculateMotorSpeed(powerL, voltage);
            powerR = calculateMotorSpeed(powerR, voltage);

            // Set power to the motors
            shooterL.setPower(powerL);
            shooterR.setPower(powerR);

            // Feed intake if close to shoot speed and waited a little
            if (runtime.seconds() > SHOOT_WAIT) {

                intake.intake(true, false);

            }

            // Only calculate after CALC_TIME_INTERVAL seconds
            if (timeDiff > CALC_TIME_INTERVAL) {

                // Difference in encoder position since last time
                double posLDiff = shooterL.getCurrentPosition() - lastPosL;
                double posRDiff = shooterR.getCurrentPosition() - lastPosR;

                // Set last time for next calculations
                lastTime = timer.seconds();
                lastPosL = shooterL.getCurrentPosition();
                lastPosR = shooterR.getCurrentPosition();

                // Calculate speed in m/s
                double speedLTPS = (posLDiff / timeDiff) * GEAR_RATIO;
                double speedLRPS = speedLTPS * L_REV_PER_TICKS;
                double speedLMPS = speedLRPS * WHEEL_CIRCUMFERENCE;
                double speedRTPS = (posRDiff / timeDiff) * GEAR_RATIO;
                double speedRRPS = speedRTPS * R_REV_PER_TICKS;
                double speedRMPS = speedRRPS * WHEEL_CIRCUMFERENCE;

                // Data to send to telemetry
                print("Time difference", timeDiff);
                print("Left motor position", shooterL.getCurrentPosition());
                print("Right motor position", shooterR.getCurrentPosition());
                print("Left position difference", posLDiff);
                print("Right position difference", posRDiff);
                print("Power L: ", powerL);
                print("Power R: ", powerR);
                print("Left motor speed (m/s): ", speedLMPS);
                print("Right motor speed (m/s): ", speedRMPS);

            }

        }

        // Stop intake
        intake.stop();

        // Stop shoot
        stopShoot();

    }

    public void shoot(boolean button, double voltage) {

        if (button) {

            // Open the stopper after a second of shooting
            stopper.setPosition(OPEN_POS);

            // Difference in time since last time
            double timeDiff = timer.seconds() - lastTime;

            // Calculate shooter power
            double powerL = calculateMotorSpeed(SHOOT_POW_L, voltage);
            double powerR = calculateMotorSpeed(SHOOT_POW_R, voltage);

            // Set power to the motors
            shooterL.setPower(powerL);
            shooterR.setPower(powerR);

            // Only calculate after CALC_TIME_INTERVAL seconds
            if (timeDiff > CALC_TIME_INTERVAL) {

                // Difference in encoder position since last time
                double posLDiff = shooterL.getCurrentPosition() - lastPosL;
                double posRDiff = shooterR.getCurrentPosition() - lastPosR;

                // Set last time for next calculations
                lastTime = timer.seconds();
                lastPosL = shooterL.getCurrentPosition();
                lastPosR = shooterR.getCurrentPosition();

                // Calculate speed in m/s
                double speedLTPS = (posLDiff / timeDiff) * GEAR_RATIO;
                double speedLRPS = speedLTPS * L_REV_PER_TICKS;
                double speedLMPS = speedLRPS * WHEEL_CIRCUMFERENCE;

                double speedRTPS = (posRDiff / timeDiff) * GEAR_RATIO;
                double speedRRPS = speedRTPS * R_REV_PER_TICKS;
                double speedRMPS = speedRRPS * WHEEL_CIRCUMFERENCE;

                // Data to send to telemetry
                print("Time difference", timeDiff);
                print("Left motor position", shooterL.getCurrentPosition());
                print("Right motor position", shooterR.getCurrentPosition());
                print("Left position difference", posLDiff);
                print("Right position difference", posRDiff);
                print("Power L: ", powerL);
                print("Power R: ", powerR);
                print("Left motor speed (m/s): ", speedLMPS);
                print("Right motor speed (m/s): ", speedRMPS);

            }

        } else {

            // If not shooting, close the stopper
            stopper.setPosition(STOP_POS);

            // If button is not pressed, stop shooting
            stopShoot();

            // Set last time for next calculations
            lastTime = timer.seconds();
            lastPosL = shooterL.getCurrentPosition();
            lastPosR = shooterR.getCurrentPosition();

        }

    }

    public void stopShoot() {

        shooterL.setPower(0);
        shooterR.setPower(0);
        stopper.setPosition(STOP_POS);

    }

}
