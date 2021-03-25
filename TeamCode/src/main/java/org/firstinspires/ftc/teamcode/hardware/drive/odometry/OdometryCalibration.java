package org.firstinspires.ftc.teamcode.hardware.drive.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

import java.io.File;

@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends LinearOpMode {

    // Drive motors
    private DcMotor right_front, right_back, left_front, left_back;

    // Odometry Wheels
    private DcMotor verticalLeft, verticalRight, horizontal;

    // Gyro
    private Gyro gyro;

    private final double PIVOT_SPEED = 0.5;

    // The amount of encoder ticks for each inch the robot moves
    private final double COUNTS_PER_INCH = 307.699557;

    private ElapsedTime timer = new ElapsedTime();

    private double horizontalTickOffset = 0;

    // Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware map values
        initHardwareMap();

        // Initialize gyro
        gyro = new Gyro(hardwareMap);

        // Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        // Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(gyro.getAngleDegrees() < 90 && opModeIsActive()){

            right_front.setPower(-PIVOT_SPEED);
            right_back.setPower(-PIVOT_SPEED);
            left_front.setPower(PIVOT_SPEED);
            left_back.setPower(PIVOT_SPEED);
            if (gyro.getAngleDegrees() < 60) {

                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);

            } else {

                setPowerAll(-PIVOT_SPEED / 2, -PIVOT_SPEED / 2, PIVOT_SPEED / 2, PIVOT_SPEED / 2);

            }

            telemetry.addData("IMU Angle", gyro.getAngleDegrees());
            telemetry.update();
        }

        // Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while (timer.milliseconds() < 1000 && opModeIsActive()) {

            telemetry.addData("IMU Angle", gyro.getAngleDegrees());
            telemetry.update();

        }

        // Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = gyro.getAngleDegrees();

        // Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        double encoderDifference = Math.abs(verticalLeft.getCurrentPosition()) + (Math.abs(verticalRight.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference / angle;

        double wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * COUNTS_PER_INCH);

        horizontalTickOffset = horizontal.getCurrentPosition()/Math.toRadians(gyro.getAngleDegrees());

        // Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){

            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");

            // Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            // Display raw values
            telemetry.addData("IMU Angle", gyro.getAngleDegrees());
            telemetry.addData("Vertical Left Position", -verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            // Update values
            telemetry.update();

        }

    }

    private void initHardwareMap(){

        right_front = hardwareMap.get(DcMotor.class, "frontRight");
        right_back = hardwareMap.get(DcMotor.class, "backRight");
        left_front = hardwareMap.get(DcMotor.class, "frontLeft");
        left_back = hardwareMap.get(DcMotor.class, "backLeft");

        verticalLeft = hardwareMap.get(DcMotor.class, "vlEncoder");
        verticalRight = hardwareMap.get(DcMotor.class, "vrEncoder");
        horizontal = hardwareMap.get(DcMotor.class, "hEncoder");

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();

    }

    private void setPowerAll(double rf, double rb, double lf, double lb){

        right_front.setPower(rf);
        right_back.setPower(rb);
        left_front.setPower(lf);
        left_back.setPower(lb);

    }

}
