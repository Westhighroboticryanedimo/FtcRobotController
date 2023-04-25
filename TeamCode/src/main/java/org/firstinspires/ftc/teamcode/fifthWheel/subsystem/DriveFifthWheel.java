package org.firstinspires.ftc.teamcode.fifthWheel.subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class DriveFifthWheel extends HolonomicDrive {
    public double p = 0.03;
    private double i = 0;
    public double d = 0.02;


    // Autonomous
    public DriveFifthWheel(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    // Teleop
    public DriveFifthWheel(OpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    // PID setup
    private void setup() {
        reduceTurn = true;
        isDrivePOV = false;
        isSquaredInputs = true;

        setPidTurn(p, i, d);
        setPidAutoSpeed(0.05, 0.001, 0);
        setPidAutoTurn(0.05 , 0.001, 0);

        setMotorDir(false, true, false, true);
        setWheelDiameter(3.7795);
        setTicksPerRev(537.7);
    }

    public void updatePID() {
        setPidTurn(p, i, d);
    }


}
