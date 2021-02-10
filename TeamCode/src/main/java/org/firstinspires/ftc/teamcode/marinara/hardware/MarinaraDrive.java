package org.firstinspires.ftc.teamcode.marinara.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class MarinaraDrive extends HolonomicDrive {

    // For teleop
    public MarinaraDrive(OpMode opMode, HardwareMap hwMap) {

        super(opMode, hwMap);
        setup();

    }

    // For autonomous
    public MarinaraDrive(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode, hwMap);
        setup();

    }

    // Setup
    private void setup() {

        // PID Values
        setPidDrive(0.05, 0, 0);
        setPidSpeed(0.05, 0.001, 0);
        setPidTurn(0.03, 0.001, 0);

        // Robot characteristics
        setTrackWidth(12);
        setWheelDiameter(4);
        setTicksPerRev(1120);

    }

}
