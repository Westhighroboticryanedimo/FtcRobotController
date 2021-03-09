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
    private static final double MAX_RPM = 1620;
    private static final double GEAR_RATIO = 80.0 / 32.0;
    private static final double WHEEL_DIAMETER_IN = 4;
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_IN * METERS_PER_INCHES * Math.PI;
    private static final double MAX_MPS = (MAX_RPM / 60) * WHEEL_CIRCUMFERENCE * GEAR_RATIO;

    // Variables for SUVAT
    private static final double Y_ACCELERATION = -9.81;
    private static final double Y_DISPLACEMENT = 0.9;
    private static final double SHOOT_ANGLE = 35;

    // Motors
    private DcMotor shooterL = null;
    private DcMotor shooterR = null;

    // Servo
    private Servo stopper = null;
    private static final double STOP_POS = 0.45;
    private static final double OPEN_POS = 0;

    // Timer
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime shootDelayer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    // Variables to use when calculating speed
    private double lastTime = 0;
    private long lastPosL = 0;
    private long lastPosR = 0;
    private static final double CALC_TIME_INTERVAL = 0.0005;
    private static final double AUTO_SHOOT_TIME = 6;

    // Misc variables
    private static final double DEFAULT_SPEED = 5.0;
    private static final double SHOOT_WAIT = 2;

    // True if over the speed
    private boolean isFeed = false;

    // PIDs
    private PIDController shooterLPID = new PIDController(0.006, 0.005, 0.02);
    private PIDController shooterRPID = new PIDController(0.006, 0.005, 0.01);

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

    private double calculateGoalSpeed(double xDisplacement) {

        // Calculate velocities
        final double X_INITIAL_VELOCITY = Math.sqrt((Y_ACCELERATION * Math.pow(xDisplacement, 2)) / (2 * (Y_DISPLACEMENT - xDisplacement * Math.tan(SHOOT_ANGLE * Math.PI / 180))));
        final double Y_INITIAL_VELOCITY = X_INITIAL_VELOCITY * Math.tan(SHOOT_ANGLE * Math.PI / 180);

        print("X Vel: ", X_INITIAL_VELOCITY);
        print("Y Vel: ", Y_INITIAL_VELOCITY);

        // Calculate net velocity using pythagorean theorem
        return Math.sqrt(Math.pow(X_INITIAL_VELOCITY, 2) + Math.pow(Y_INITIAL_VELOCITY, 2));

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

    public void shoot(double powerL, double powerR, Intake intake) {

        runtime.reset();
        lastTime = timer.seconds();
        while (linearOpMode.opModeIsActive() && runtime.seconds() < AUTO_SHOOT_TIME) {

            // Open shooter
            stopper.setPosition(OPEN_POS);

            // Difference in time since last time
            double timeDiff = timer.seconds() - lastTime;

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

                double goalSpeed = 6;

                // Setup PID
                shooterLPID.setInputRange(0, MAX_MPS);
                shooterRPID.setInputRange(0, MAX_MPS);
                shooterLPID.setOutputRange(0, 1);
                shooterRPID.setOutputRange(0, 1);
                shooterLPID.setSetpoint(goalSpeed + 2);
                shooterRPID.setSetpoint(goalSpeed - 2);
                shooterLPID.enable();
                shooterRPID.enable();

                // Calculate PID
                //double powerL = shooterLPID.performPID(speedLMPS);
                //double powerR = shooterRPID.performPID(speedRMPS);

                // Set power to the motors
                shooterL.setPower(powerL);
                shooterR.setPower(powerR);
/*
                if (((speedLMPS + speedRMPS) / 2) > (goalSpeed - 0.5) && ((speedLMPS + speedRMPS) / 2) < goalSpeed + 0.5) {

                    intake.intake(true, false);

                }

 */

                // Feed intake if close to shoot speed and waited a little
                if (/*(speedLMPS + speedRMPS) / 2 > goalSpeed - 0.5 && (speedLMPS + speedRMPS) / 2 < goalSpeed + 0.5 && */runtime.seconds() > SHOOT_WAIT) {

                    intake.intake(true, false);

                }

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

        // Disable PID
        shooterLPID.disable();
        shooterRPID.disable();

    }

    public void shoot(boolean button, float[] displacement) {

        if (button) {

            // Open the stopper after a second of shooting
            stopper.setPosition(OPEN_POS);

            // Calculate displacement
            double totalDisplacement = Math.sqrt(Math.pow(displacement[0], 2) + Math.pow(displacement[1], 2)) * METERS_PER_INCHES;

            // The goal speed in m/s (currently inactive!)
            double goalSpeedMPS;
            if (totalDisplacement == 0) {

                goalSpeedMPS = DEFAULT_SPEED;

            } else {

                goalSpeedMPS = calculateGoalSpeed(totalDisplacement);

            }

            // Forcefully set m/s
            goalSpeedMPS = 6.1;

            // Difference in time since last time
            double timeDiff = timer.seconds() - lastTime;

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

                // Setup PID
                shooterLPID.setInputRange(0, MAX_MPS);
                shooterRPID.setInputRange(0, MAX_MPS);
                shooterLPID.setOutputRange(0, 1);
                shooterRPID.setOutputRange(0, 1);
                shooterLPID.setSetpoint(goalSpeedMPS + 2);
                shooterRPID.setSetpoint(goalSpeedMPS - 2);
                shooterLPID.enable();
                shooterRPID.enable();

                // Calculate PID
                double powerL = shooterLPID.performPID(speedLMPS);
                double powerR = shooterRPID.performPID(speedRMPS);

                powerL = 0.55;
                powerR = 0.33;

                // Set power to the motors
                shooterL.setPower(powerL);
                shooterR.setPower(powerR);

                // Feed intake if close to shoot speed and more than one second passed
                //isFeed = ((speedLMPS + speedRMPS) / 2) > (goalSpeedMPS - 0.5) && ((speedLMPS + speedRMPS) / 2) < goalSpeedMPS + 0.5;
                //isFeed = /*(speedLMPS + speedRMPS) / 2 > goalSpeedMPS - 0.5 && (speedLMPS + speedRMPS) / 2 < goalSpeedMPS + 0.5 &&*/ shootDelayer.seconds() > SHOOT_WAIT;

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
                print("Goal speed (m/s): ", goalSpeedMPS);
                print("Total Displacement: ", totalDisplacement);

            }

        } else {

            // If not shooting, close the stopper
            stopper.setPosition(STOP_POS);

            // If button is not pressed, stop shooting
            stopShoot();

            // Disable PID
            shooterLPID.disable();
            shooterRPID.disable();

            // Set last time for next calculations
            lastTime = timer.seconds();
            lastPosL = shooterL.getCurrentPosition();
            lastPosR = shooterR.getCurrentPosition();

            // Don't feed if not shooting
            isFeed = false;

            // Reset time
            shootDelayer.reset();

        }

    }

    public boolean getIsFeed() {

        return isFeed;

    }

    public void stopShoot() {

        shooterL.setPower(0);
        shooterR.setPower(0);
        stopper.setPosition(STOP_POS);

    }

}
