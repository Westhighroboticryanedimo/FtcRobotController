package org.firstinspires.ftc.teamcode.ultimategoal.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;

public class Shooter extends BaseHardware {

    // Final variables that won't change; used for calculations
    private static final double MAX_RPM = 1620;
    private static final double WHEEL_DIAMETER_IN = 4;
    private static final double WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER_IN / 39.37) * Math.PI;
    private static final double MAX_MPS = (MAX_RPM / 60) * WHEEL_CIRCUMFERENCE;
    private static final double TICKS_PER_REV = 103.6;
    private static final double REV_PER_TICKS = 1 / TICKS_PER_REV;

    // Variables for SUVAT
    private static final double Y_ACCELERATION = -9.81;
    private static final double Y_DISPLACEMENT = 2;
    private static final double SHOOT_ANGLE = 35;

    // Motors
    private DcMotor shooterL = null;
    private DcMotor shooterR = null;

    // Timer
    private ElapsedTime timer = new ElapsedTime();

    // Variables to use when calculating speed
    private double lastTime = 0;
    private double timeDiff = 0;
    private double powerL = 0;
    private double powerR = 0;
    private long lastPosL = 0;
    private long lastPosR = 0;
    private long posLDiff = 0;
    private long posRDiff = 0;
    private double speedLMPS = 0;
    private double speedRMPS = 0;
    private static final double CALC_TIME_INTERVAL = 0.020;


    // PIDs
    private PIDController shooterLPID = new PIDController(0.00000001, 0.005, 0.1);
    private PIDController shooterRPID = new PIDController(0.00000001, 0.005, 0.1);

    // Teleop constructor
    public Shooter(OpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setupMotor(hwMap);

    }

    // Autonomous constructor
    public Shooter(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);
        setupMotor(hwMap);

    }

    private void setupMotor(HardwareMap hwMap) {

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

    }

    private double calculateGoalSpeed(double xDisplacement) {

        // Calculate velocities
        final double X_INITIAL_VELOCITY = Math.sqrt((Y_ACCELERATION * Math.pow(xDisplacement, 2)) / (2 * (Y_DISPLACEMENT - xDisplacement * Math.tan(SHOOT_ANGLE))));
        final double Y_INITIAL_VELOCITY = X_INITIAL_VELOCITY * Math.tan(SHOOT_ANGLE);

        // Calculate net velocity using pythagorean theorem
        final double NET_VELOCITY = Math.sqrt(Math.pow(X_INITIAL_VELOCITY, 2) + Math.pow(Y_INITIAL_VELOCITY, 2));

        return NET_VELOCITY;

    }

    @Override
    public void update() {

        // If debug mode is on, add motor speed to telemetry
        if (isDebugMode) {

            opMode.telemetry.addData("Time diff: ", timeDiff);
            opMode.telemetry.addData("PosL Diff: ", posLDiff);
            opMode.telemetry.addData("PosR Diff: ", posRDiff);
            opMode.telemetry.addData("Power L: ", powerL);
            opMode.telemetry.addData("Power R: ", powerR);
            opMode.telemetry.addData("Left motor speed (m/s): ", speedLMPS);
            opMode.telemetry.addData("Right motor speed (m/s): ", speedRMPS);

        }

    }

    public void shoot(boolean button) {

        if (button) {

            // If the button if pressed, shoot

            // The goal speed in m/s
            double goalSpeedMPS = 5;

            // Difference in time since last time
            timeDiff = timer.seconds() - lastTime;

            // Only calculate after CALC_TIME_INTERVAL seconds
            if (timeDiff > CALC_TIME_INTERVAL) {

                // Difference in encoder position since last time
                posLDiff = shooterL.getCurrentPosition() - lastPosL;
                posRDiff = shooterR.getCurrentPosition() - lastPosR;

                // Set last time for next calculations
                lastTime = timer.seconds();
                lastPosL = shooterL.getCurrentPosition();
                lastPosR = shooterR.getCurrentPosition();

                // Calculate speed in m/s
                double speedLTPS = posLDiff / timeDiff;
                double speedLRPS = speedLTPS * REV_PER_TICKS;
                speedLMPS = speedLRPS * WHEEL_CIRCUMFERENCE;

                double speedRTPS = posRDiff / timeDiff;
                double speedRRPS = speedRTPS * REV_PER_TICKS;
                speedRMPS = speedRRPS * WHEEL_CIRCUMFERENCE;

                // Setup PID
                shooterLPID.setSetpoint(goalSpeedMPS);
                shooterRPID.setSetpoint(goalSpeedMPS);
                shooterLPID.setInputRange(0, MAX_MPS);
                shooterRPID.setInputRange(0, MAX_MPS);
                shooterLPID.setOutputRange(0, 1);
                shooterRPID.setOutputRange(0, 1);
                shooterLPID.enable();
                shooterRPID.enable();

                // Calculate PID
                powerL = shooterLPID.performPID(speedLMPS);
                powerR = shooterRPID.performPID(speedRMPS);

                // Set power to the motors
                shooterL.setPower(powerL);
                shooterR.setPower(powerR);

            }

        } else {

            // If button is not pressed, stop shooting
            shooterL.setPower(0);
            shooterR.setPower(0);

            // Set last time for next calculations
            lastTime = timer.seconds();
            lastPosL = shooterL.getCurrentPosition();
            lastPosR = shooterR.getCurrentPosition();

        }

    }

}
