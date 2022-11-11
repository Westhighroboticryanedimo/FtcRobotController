package org.firstinspires.ftc.teamcode.IF;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class MFDrive extends HolonomicDrive {


    //Autonomous code
    public MFDrive(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    //Teleop code
    public MFDrive(OpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    //PID setup
    private void setup() {
        setPidDrive(0.035,0,0.01);
        setPidSpeed(0.05, 0.001, 0);
        setPidTurn(0.028, 0.001, 0);

        setMotorDir(false, true, true, false);
        setWheelDiameter(3.7795);
        setTicksPerRev(537.7);
        setPIDFalse();
    }
}
