package org.firstinspires.ftc.teamcode.marinara.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class MarinaraDrive extends HolonomicDrive {

    private ColorSensor color;
    private DistanceSensor distanceRight;
    private DistanceSensor distanceBack;
    private static final int BLUE_THRESHOLD = 100;

    // For teleop
    public MarinaraDrive(OpMode opMode, HardwareMap hwMap) {

        super(opMode, hwMap);
        setup(hwMap);

    }

    // For autonomous
    public MarinaraDrive(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode, hwMap);
        setup(hwMap);

    }

    // Setup
    private void setup(HardwareMap hwMap) {

        // PID Values
        setPidDrive(0.02, 0, 0);
        setPidSpeed(0.03, 0.002, 0);
        setPidTurn(0.03, 0.001, 0);

        // Robot characteristics
        setWheelDiameter(3.9);
        setTicksPerRev(537.6);

        // Color sensor on bot
        color = hwMap.get(ColorSensor.class, "color");

        // Distance sensors on bot
        distanceBack = hwMap.get(DistanceSensor.class, "distanceBack");
        distanceRight = hwMap.get(DistanceSensor.class, "distanceRight");

    }

    public void moveUntilDistanceBack(double speed, double angleMove, double toWallDistance) {

        // Reset motors and gyro
        gyro.reset();
        resetMotors();

        // Calculate distance of motors
        double flbrDist = Math.sin(Math.toRadians(45.0 + angleMove));
        double frblDist = Math.cos(Math.toRadians(45.0 + angleMove));

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
        pidFLBR.setInputRange(0, getDistanceBack());
        pidFLBR.setOutputRange(0, goalFLBRSpeed);
        pidFLBR.setSetpoint(toWallDistance);
        pidFLBR.setTolerance(3);
        pidFLBR.enable();

        // For frontright and backleft motors
        pidFRBL.reset();
        pidFRBL.setInputRange(0, getDistanceBack());
        pidFRBL.setOutputRange(0, goalFRBLSpeed);
        pidFRBL.setSetpoint(toWallDistance);
        pidFRBL.setTolerance(3);
        pidFRBL.enable();

        do {

            correction = pidDrive.performPID(gyro.getAngleDegrees());

            double speedFLBR = pidFLBR.performPID(getDistanceBack());
            double speedFRBL = pidFRBL.performPID(getDistanceBack());

            frontLeft.setPower(speedFLBR - correction);
            frontRight.setPower(speedFRBL + correction);
            backLeft.setPower(speedFRBL - correction);
            backRight.setPower(speedFLBR + correction);

            print("Correction: ", correction);
            print("FLBR Speed: ", speedFLBR);
            print("FRBL Speed: ", speedFRBL);

            linearOpMode.telemetry.update();

        } while (linearOpMode.opModeIsActive() && (!pidFRBL.onTarget() || !pidFLBR.onTarget()));

        stop();

        linearOpMode.sleep(100);

    }

    public void moveUntilDistanceRight(double speed, double angleMove, double toWallDistance) {

        // Reset motors and gyro
        gyro.reset();
        resetMotors();

        // Calculate distance of motors
        double flbrDist = Math.sin(Math.toRadians(45.0 + angleMove));
        double frblDist = Math.cos(Math.toRadians(45.0 + angleMove));

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
        pidFLBR.setInputRange(0, getDistanceRight());
        pidFLBR.setOutputRange(0, goalFLBRSpeed);
        pidFLBR.setSetpoint(toWallDistance);
        pidFLBR.setTolerance(3);
        pidFLBR.enable();

        // For frontright and backleft motors
        pidFRBL.reset();
        pidFRBL.setInputRange(0, getDistanceRight());
        pidFRBL.setOutputRange(0, goalFRBLSpeed);
        pidFRBL.setSetpoint(toWallDistance);
        pidFRBL.setTolerance(3);
        pidFRBL.enable();

        do {

            correction = pidDrive.performPID(gyro.getAngleDegrees());

            double speedFLBR = pidFLBR.performPID(getDistanceRight());
            double speedFRBL = pidFRBL.performPID(getDistanceRight());

            frontLeft.setPower(-speedFLBR - correction);
            frontRight.setPower(speedFRBL + correction);
            backLeft.setPower(speedFRBL - correction);
            backRight.setPower(-speedFLBR + correction);

            print("Correction: ", correction);
            print("FLBR Speed: ", speedFLBR);
            print("FRBL Speed: ", speedFRBL);

            linearOpMode.telemetry.update();

        } while (linearOpMode.opModeIsActive() && (!pidFRBL.onTarget() || !pidFLBR.onTarget()));

        stop();

        linearOpMode.sleep(100);

    }

    public int[] getColors() {

        return new int[] { color.red(), color.green(), color.blue() };

    }

    public double getDistanceBack() {

        return distanceBack.getDistance(DistanceUnit.INCH);

    }

    public double getDistanceRight() {

        return distanceRight.getDistance(DistanceUnit.INCH);

    }

    // Check color sensor status
    public class ColorCommand implements BoolCommand {

        @Override
        public boolean check() {

            return getColors()[2] > BLUE_THRESHOLD;

        }

    }

    // Check if right distance is over a certain threshold
    public class DistanceRightCommand implements BoolCommand {

        private final double distance;

        public DistanceRightCommand(double distance) {

            this.distance = distance;

        }

        @Override
        public boolean check() {

            return distance > getDistanceRight();

        }

    }

    // Check if back distance is over a certain threshold
    public class DistanceBackCommand implements BoolCommand {

        private final double distance;

        public DistanceBackCommand(double distance) {

            this.distance = distance;

        }

        @Override
        public boolean check() {

            return distance > getDistanceBack();

        }

    }

}
