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

    // Motors
    private DcMotor shooterL = null;
    private DcMotor shooterR = null;

    // Servo
    private Servo stopper = null;
    private static final double STOP_POS = 0.4;
    private static final double OPEN_POS = 0;

    // Timer
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    // Variables to use when calculating speed
    private double lastTime = 0;
    private long lastPosL = 0;
    private long lastPosR = 0;
    private static final double AUTO_SHOOT_TIME = 6;

    // Misc variables
    private static final double SHOOT_WAIT = 2;
    private static final double SHOOT_POW = 0.86;
    private static final double SHOOT_DIFF = 0.5;

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

    private double calculateMotorSpeed(double voltage) {

        final double FINE_JUST_FOR_ARELI = 0.1;

        // Quadratic relationship
        return 4.53 - 0.556 * voltage + 0.0207 * Math.pow(voltage, 2) + FINE_JUST_FOR_ARELI;

    }

    public void shoot(double voltage, Intake intake) {

        runtime.reset();
        lastTime = timer.seconds();
        while (linearOpMode.opModeIsActive() && runtime.seconds() < AUTO_SHOOT_TIME) {

            // Open shooter
            stopper.setPosition(OPEN_POS);

            // Calculate motor speeds
            double p = calculateMotorSpeed(voltage);

            // Set power to the motors
            shooterL.setPower(p);
            shooterR.setPower(p * SHOOT_DIFF);

            // Feed intake if close to shoot speed and waited a little
            if (runtime.seconds() > SHOOT_WAIT) {

                intake.intake(true, false);

            }

            // Data to send to telemetry
            print("Left motor position", shooterL.getCurrentPosition());
            print("Right motor position", shooterR.getCurrentPosition());

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

            // Calculate shooter power
            double powerL = calculateMotorSpeed(voltage);
            double powerR = powerL * SHOOT_DIFF;

            // Set power to the motors
            shooterL.setPower(powerL);
            shooterR.setPower(powerR);

            // Data to send to telemetry
            print("Left motor position", shooterL.getCurrentPosition());
            print("Right motor position", shooterR.getCurrentPosition());
            print("Power L: ", powerL);
            print("Power R: ", powerR);

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
