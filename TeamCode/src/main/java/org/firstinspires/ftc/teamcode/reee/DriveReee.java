package org.firstinspires.ftc.teamcode.reee;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class DriveReee extends HolonomicDrive {


    // Autonomous
    public DriveReee(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    // Teleop
    public DriveReee(OpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    // PID setup
    private void setup() {
        thirdWheel = false;
        reduceTurn = false;
        isDrivePOV = false;


        setPidTurn(0.035, 0, 0.01);
        setPidAutoSpeed(0.05, 0.001, 0);
        setPidAutoTurn(0.028, 0.001, 0);

        setMotorDir(false, true, false, true);
        setWheelDiameter(3.7795);
        setTicksPerRev(537.7);
    }

}
