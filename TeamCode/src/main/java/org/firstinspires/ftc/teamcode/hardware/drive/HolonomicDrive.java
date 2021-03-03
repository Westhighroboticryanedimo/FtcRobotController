package org.firstinspires.ftc.teamcode.hardware.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.hardware.BaseHardware;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

public abstract class HolonomicDrive extends BaseHardware {

    // Boolean function for moveUntil()
    public interface BoolCommand {

        boolean check();

    }

    // Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Gyro gyro;

    private boolean isDrivePOV = false;

    // For autonomous driving
    private double wheelDiameter = 4;   // Diameter of driving wheels
    private double ticksPerRev = 1120;  // TPR of driving motors

    // For PID corrections
    private PIDController pidDrive = new PIDController(0, 0, 0);
    private PIDController pidFLBR = new PIDController(0, 0, 0);
    private PIDController pidFRBL = new PIDController(0, 0, 0);
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

        pidFLBR = new PIDController(p, i, d);
        pidFRBL = new PIDController(p, i, d);

    }

    // Set pidTurn values
    protected void setPidTurn(double p, double i, double d) {

        pidTurn = new PIDController(p, i, d);

    }

    // Call this function to stop correction
    protected void setPIDFalse() {

        isPID = false;

    }


    protected void setMotorDir(boolean flDir, boolean frDir, boolean blDir, boolean brDir) {

        if (flDir) frontLeft.setDirection(DcMotor.Direction.FORWARD);
        else frontLeft.setDirection(DcMotor.Direction.REVERSE);

        if (frDir) frontRight.setDirection(DcMotor.Direction.FORWARD);
        else frontRight.setDirection(DcMotor.Direction.REVERSE);

        if (blDir) backLeft.setDirection(DcMotor.Direction.FORWARD);
        else backLeft.setDirection(DcMotor.Direction.REVERSE);

        if (brDir) backRight.setDirection(DcMotor.Direction.FORWARD);
        else backRight.setDirection(DcMotor.Direction.REVERSE);

    }

    // Set the wheel diameter
    protected void setWheelDiameter(double wheelDiameter) {

        this.wheelDiameter = wheelDiameter;

    }

    // Set ticks per revolution
    protected void setTicksPerRev(double ticksPerRev) {

        this.ticksPerRev = ticksPerRev;

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

    public boolean isInPOVMode() {
        return isDrivePOV;
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

        // PID calculation
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

        print("Angle: ", robotAngle * 180 / Math.PI);

        // Takes the x and y components of the vector
        double cosinePow = r * Math.cos(robotAngle);
        double sinePow = r * Math.sin(robotAngle);

        print("Sinepower: ", sinePow);

        // Takes the bigger number and multiplies it by a ratio to speed up the robot
        double maxPow = Math.max(Math.abs(cosinePow), Math.abs(sinePow));
        double ratio = 0;
        if (maxPow != 0) ratio = r / maxPow;

        print("Ratio: ", ratio);

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

        // Telemetry values
        print("FLPow: ", frontLeft.getPower());
        print("FRPow: ", frontRight.getPower());
        print("BLPow: ", backLeft.getPower());
        print("BRPow: ", backRight.getPower());
        //print("FL: ", frontLeft.getCurrentPosition());
        //print("FR: ", frontRight.getCurrentPosition());
        //print("BL: ", backLeft.getCurrentPosition());
        //print("BR: ", backRight.getCurrentPosition());
        print("Gyro: ", gyro.getAngleDegrees());

    }

    public void moveUntil(double speed, double angleMove, BoolCommand command) {

        gyro.reset();
        resetMotors();

        linearOpMode.sleep(100);

        // For correction turning
        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.enable();

        // Calculate speed
        double flbrSpeed = speed * Math.sin((45.0 + angleMove) * Math.PI / 180);
        double frblSpeed = speed * Math.cos((45.0 + angleMove) * Math.PI / 180);

        // Maximize speed by calculating ratio
        double max = Math.max(Math.abs(flbrSpeed), Math.abs(frblSpeed));
        flbrSpeed /= max;
        frblSpeed /= max;

        do {

            correction = pidDrive.performPID(gyro.getAngleDegrees());

            frontLeft.setPower(flbrSpeed - correction);
            frontRight.setPower(frblSpeed + correction);
            backLeft.setPower(frblSpeed - correction);
            backRight.setPower(flbrSpeed+ correction);

            print("Gyro: ", gyro.getAngleDegrees());
            print("Boolean value: ", command.check());
            print("Correction: ", correction);

            linearOpMode.telemetry.update();

        } while (linearOpMode.opModeIsActive() && !command.check());

        stop();

    }

    public void move(double speed, double distance, double angleMove) {

        gyro.reset();
        resetMotors();

        linearOpMode.sleep(100);

        // The number of ticks the motors have to move without considering direction of motors
        int goalTicks = (int) ((distance / (wheelDiameter * Math.PI)) * ticksPerRev);

        // Calculate distance of motors
        int flbrDist = (int) (goalTicks * Math.sin((45.0 + angleMove) * Math.PI / 180));
        int frblDist = (int) (goalTicks * Math.cos((45.0 + angleMove) * Math.PI / 180));

        // Calculate speeds
        double max = Math.max(Math.abs(flbrDist), Math.abs(frblDist));
        double goalFLBRSpeed = speed * (flbrDist / max);
        double goalFRBLSpeed = speed * (frblDist / max);

        // For correction turning
        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.enable();

        // For frontleft and backright motors
        pidFLBR.reset();
        pidFLBR.setInputRange(0, flbrDist);
        pidFLBR.setOutputRange(0, goalFLBRSpeed);
        pidFLBR.setSetpoint(flbrDist);
        pidFLBR.setTolerance(0.8);
        pidFLBR.enable();

        // For frontright and backleft motors
        pidFRBL.reset();
        pidFRBL.setInputRange(0, frblDist);
        pidFRBL.setOutputRange(0, goalFRBLSpeed);
        pidFRBL.setSetpoint(frblDist);
        pidFRBL.setTolerance(0.5);
        pidFRBL.enable();

        boolean isFLBROnTarget = false;
        boolean isFRBLOnTarget = false;
        if (flbrDist == 0) isFLBROnTarget = true;
        else if (frblDist == 0) isFRBLOnTarget = true;

        do {

            correction = pidDrive.performPID(gyro.getAngleDegrees());

            double avgFLBRPos = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 2.0;
            double speedFLBR = pidFLBR.performPID(avgFLBRPos);

            double avgFRBLPos = (frontRight.getCurrentPosition() + backLeft.getCurrentPosition()) / 2.0;
            double speedFRBL = pidFRBL.performPID(avgFRBLPos);

            frontLeft.setPower(speedFLBR - correction);
            frontRight.setPower(speedFRBL + correction);
            backLeft.setPower(speedFRBL - correction);
            backRight.setPower(speedFLBR + correction);

            //print("Gyro: ", gyro.getAngleDegrees());
            //print("FL: ", frontLeft.getCurrentPosition());
            //print("FR: ", frontRight.getCurrentPosition());
            //print("BL: ", backLeft.getCurrentPosition());
            //print("BR: ", backRight.getCurrentPosition());
            print("Correction: ", correction);
            print("Average FLBR: ", avgFLBRPos);
            print("Average FRBL: ", avgFRBLPos);
            print("FLBR Setpoint: ", pidFLBR.getSetpoint() + " " + Double.toString(flbrDist));
            print("FRBL Setpoint: ", pidFRBL.getSetpoint() + " " + Double.toString(frblDist));
            print("FLBR Speed: ", speedFLBR);
            print("FRBL Speed: ", speedFRBL);
            print("FLBR Ontarget: ", pidFLBR.onTarget() || isFLBROnTarget);
            print("FRBL Ontarget: ", pidFRBL.onTarget() || isFRBLOnTarget);

            linearOpMode.telemetry.update();

        } while (linearOpMode.opModeIsActive() && ((!pidFRBL.onTarget() && !isFRBLOnTarget) || (!pidFLBR.onTarget() && !isFLBROnTarget)));

        stop();

    }

    public void turn(double speed, int angle) {

        gyro.reset();
        resetMotors();

        if (Math.abs(angle) > 359) angle = (int) Math.copySign(359, angle);

        pidTurn.reset();
        pidTurn.setInputRange(0, -angle);
        pidTurn.setOutputRange(0, speed);
        pidTurn.setSetpoint(-angle);
        pidTurn.setTolerance(0.5);
        pidTurn.enable();

        linearOpMode.sleep(100);

        do {

            speed = pidTurn.performPID(gyro.getAngleDegrees());

            frontLeft.setPower(-speed);
            frontRight.setPower(speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);

            print("FL: ", frontLeft.getCurrentPosition());
            print("FR: ", frontRight.getCurrentPosition());
            print("BL: ", backLeft.getCurrentPosition());
            print("BR: ", backRight.getCurrentPosition());

            linearOpMode.telemetry.update();

        } while (linearOpMode.opModeIsActive() && !pidTurn.onTarget());

        stop();

    }

    public void stop() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    public void resetMotors() {

        stop();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

}
