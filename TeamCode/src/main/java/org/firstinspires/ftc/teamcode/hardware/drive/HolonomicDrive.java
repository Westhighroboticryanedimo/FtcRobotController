package org.firstinspires.ftc.teamcode.hardware.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

public class HolonomicDrive extends BaseHardware {

    // Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Gyro gyro;

    private boolean isDrivePOV = true;

    // For PID corrections
    private PIDController pidDrive = new PIDController(0, 0, 0);
    private PIDController pidSpeed = new PIDController(0, 0, 0);
    private PIDController pidTurn = new PIDController(0, 0, 0);
    private double prevAngle;
    private double correction;
    private boolean isPID = true;

    // Set pidDrive values
    protected void setPidDrive(double p, double i, double d) {

        pidDrive = new PIDController(p, i, d);

    }

    // Set pidStop values
    protected void setPidSpeed(double p, double i, double d) {

        pidSpeed = new PIDController(p, i, d);

    }

    // Set pidTurn values
    protected void setPidTurn(double p, double i, double d) {

        pidTurn = new PIDController(p, i, d);

    }

    // Call this function to stop correction
    protected void setPIDFalse() {

        isPID = false;

    }

    // For teleop
    public HolonomicDrive(OpMode opMode, HardwareMap hwMap) {

        super(opMode);

        gyro = new Gyro(hwMap);
        prevAngle = gyro.getAngleDegrees();

        setupMotors(hwMap);

    }

    // For autonomous
    public HolonomicDrive(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode);

        gyro = new Gyro(hwMap);
        prevAngle = gyro.getAngleDegrees();

        setupMotors(hwMap);

    }

    // Define and initialize motors
    private void setupMotors(HardwareMap hwMap) {

        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setPower(0);

        frontRight = hwMap.get(DcMotor.class, "frontRight");
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setPower(0);

        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setPower(0);

        backRight = hwMap.get(DcMotor.class, "backRight");
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setPower(0);

        setupEncoders();

    }

    private void setupEncoders() {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (linearOpMode == null) {

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        } else {

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

    }

    // Toggle POV mode and field oriented mode
    public void togglePOV(boolean button) {

        if (button) {

            isDrivePOV = !isDrivePOV;
            gyro.reset();
            prevAngle = gyro.getAngleDegrees();

        }

    }

    // TeleOp Drive
    public void drive(double joystickX, double joystickY, double joystickTurn) {

        // Reduce joystick turn
        joystickTurn /= 1.5;

        // pid calculation
        pidDrive.setSetpoint(prevAngle);
        correction = pidDrive.performPID(gyro.getAngleDegrees());

        // If turning or not moving then don't correct
        if (Math.abs(joystickTurn) != 0 ||
           (joystickX == 0 && joystickY == 0)) {

            pidDrive.disable();
            pidDrive.reset();
            prevAngle = gyro.getAngleDegrees();
            correction = 0;

        } else
            pidDrive.enable();

        // If field oriented drive, then set angleCompensation to gyro angle
        double angleCompensation = 0;
        if (!isDrivePOV)
            angleCompensation = gyro.getAngleRadians();

        // Holonomic drive calculations

        // The magnitude of the joystick
        double r = Math.hypot(joystickX, joystickY);

        // The angle of the robot it is supposed to move towards
        double robotAngle = Math.atan2(-joystickY, joystickX) - Math.PI / 4 - angleCompensation;

        // Takes the x and y components of the vector
        double cosinePow = r * Math.cos(robotAngle);
        double sinePow = r * Math.sin(robotAngle);

        // Takes the bigger number and multiplies it by a ratio to speed up the robot
        double maxPow = Math.max(Math.abs(cosinePow), Math.abs(sinePow));
        double ratio = 0;
        if (maxPow != 0) ratio = r / maxPow;

        // Motor powers without PID corrections
        double v1 = cosinePow * ratio + joystickTurn;
        double v2 = sinePow * ratio - joystickTurn;
        double v3 = sinePow * ratio + joystickTurn;
        double v4 = cosinePow * ratio - joystickTurn;

        // Divide by ratio to make turning easier

        double flbrMax = Math.max(Math.abs(v1), Math.abs(v4));
        if (flbrMax > 1.0) {

            v1 /= flbrMax;
            v4 /= flbrMax;

        }

        double frblMax = Math.max(Math.abs(v2), Math.abs(v3));
        if (frblMax > 1.0) {

            v2 /= frblMax;
            v3 /= frblMax;

        }

        // Don't correct if isPID is false
        if (!isPID) correction = 0;

        // Add PID corrections
        v1 -= correction;
        v2 += correction;
        v3 -= correction;
        v4 += correction;

        // Set motor powers
        frontLeft.setPower(v1);
        frontRight.setPower(v2);
        backLeft.setPower(v3);
        backRight.setPower(v4);

    }

    public void moveForward(double speed, int position) {

        gyro.reset();
        resetMotors();
        linearOpMode.sleep(100);

        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.enable();

        pidSpeed.reset();
        pidSpeed.setSetpoint(position);
        pidSpeed.setInputRange(0, position);
        pidSpeed.setOutputRange(0, speed);
        pidSpeed.setTolerance(1);
        pidSpeed.enable();

        do {

            correction = pidDrive.performPID(gyro.getAngleDegrees());

            double averagePosition = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() +
                                     backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 4.0;
            speed = pidSpeed.performPID(averagePosition);

            frontLeft.setPower(speed - correction);
            frontRight.setPower(speed + correction);
            backLeft.setPower(speed - correction);
            backRight.setPower(speed + correction);

        } while (linearOpMode.opModeIsActive() && !pidSpeed.onTarget());

    }

    public void moveSide(double speed, int position) {

        gyro.reset();
        resetMotors();
        linearOpMode.sleep(100);

        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.enable();

        pidSpeed.reset();
        pidSpeed.setSetpoint(position);
        pidSpeed.setInputRange(0, position);
        pidSpeed.setOutputRange(0, speed);
        pidSpeed.setTolerance(1);
        pidSpeed.enable();

        do {

            correction = pidDrive.performPID(gyro.getAngleDegrees());

            double averagePosition = (frontLeft.getCurrentPosition() - frontRight.getCurrentPosition() -
                    backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 4.0;
            speed = pidSpeed.performPID(averagePosition);

            frontLeft.setPower(speed - correction);
            frontRight.setPower(-speed + correction);
            backLeft.setPower(-speed - correction);
            backRight.setPower(speed + correction);

        } while (linearOpMode.opModeIsActive() && !pidSpeed.onTarget());

    }

    public void turn(double speed, int angle) {

        gyro.reset();
        resetMotors();
        linearOpMode.sleep(100);

        if (Math.abs(angle) > 359) angle = (int) Math.copySign(359, angle);

        pidTurn.reset();
        pidTurn.setSetpoint(-angle);
        pidTurn.setInputRange(0, -angle);
        pidTurn.setOutputRange(0, speed);
        pidTurn.setTolerance(1);
        pidTurn.enable();

        do {

            speed = pidTurn.performPID(gyro.getAngleDegrees());

            frontLeft.setPower(-speed);
            frontRight.setPower(speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);

        } while (linearOpMode.opModeIsActive() && !pidTurn.onTarget());

    }

    public void resetMotors() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void update() {

        if (isDebugMode) {

            opMode.telemetry.addData("FL: ", frontLeft.getCurrentPosition());
            opMode.telemetry.addData("FR: ", frontRight.getCurrentPosition());
            opMode.telemetry.addData("BL: ", backLeft.getCurrentPosition());
            opMode.telemetry.addData("BR: ", backRight.getCurrentPosition());

        }

    }

}
