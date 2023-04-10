package org.firstinspires.ftc.teamcode.NewPPRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class ChodeDrive extends HolonomicDrive {
    public double p = 0.05;
    private double i = 0.0;
    public double d = 0.0;


    // Autonomous
    public ChodeDrive(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    // Teleop
    public ChodeDrive(OpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    // PID setup
    private void setup() {
        reduceTurn = true;
        isDrivePOV = true;
        isSquaredInputs = true;

        setPidTurn(0.05, 0, 0.0);
        setPidAutoSpeed(0.028, 0.001, 0);
        setPidAutoTurn(0.028, 0.001, 0);

        setMotorDir(true, false, true, false);
        setWheelDiameter(3.7795);
        setTicksPerRev(537.7);
    }

    public void updatePID() {
        setPidTurn(p, i, d);
    }

    // Field Centric Toggle
    public void FieldCentricToggle() {
        if (isDrivePOV == false) {
            isDrivePOV = true;
        } else {
            isDrivePOV = false;
        }
    }

    //PID Tuning Methods
    public void changeDriveP(double change) {
        p = p + change;
    }
    public void changeDriveI(double change) {
        i = i + change;
    }
    public void changeDriveD(double change) {
        d = d + change;
    }
    public double getDriveP() {return(p);}
    public double getDriveI() {return(i);}
    public double getDriveD() {return(d);}


}
