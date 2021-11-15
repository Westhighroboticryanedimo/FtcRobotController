package org.firstinspires.ftc.teamcode.thirdWheel.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class DriveThirdWheel extends HolonomicDrive {


    // Autonomous
    public DriveThirdWheel(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    // Teleop
    public DriveThirdWheel(OpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
    }

    // PID setup
    private void setup() {

        setPidDrive(1, 0.01, 0);
        setPidSpeed(0.05, 0.001, 0);
        setPidTurn(0.028, 0.001, 0);

        setMotorDir(true, true, false, false);

        setWheelDiameter(3.7795);
        setTicksPerRev(537.7);
    }

}
