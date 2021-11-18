package org.firstinspires.ftc.teamcode.vampire.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class VampireDrive extends HolonomicDrive {

    // For autonomous

    private ColorSensor color;
    private DistanceSensor distanceRight;
    private DistanceSensor distanceBack;
    private static final int BLUE_THRESHOLD = 100;

    // For teleop
    public VampireDrive(OpMode opMode, HardwareMap hwMap) {

        super(opMode, hwMap);
        setup(hwMap);

    }
    public VampireDrive(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode, hwMap);
        setup(hwMap);

    }

    // Setup
    private void setup(HardwareMap hwMap) {

        // PID Values
        setPidDrive(0.05, 0, 0.009);
        setPidSpeed(0.03, 0.002, 0);
        setPidTurn(0.03, 0.001, 0);

        // Set motor direction
        setMotorDir(false, true, false, true);

        // Robot characteristics
        setWheelDiameter(3.9);
        setTicksPerRev(537.6);

    }

}
