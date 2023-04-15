package org.firstinspires.ftc.teamcode.WizardBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class WizardBotDrive extends HolonomicDrive {
    public double p = 0.04;
    private double i = 0.0;
    public double d = 0.01;


    // Autonomous
    public WizardBotDrive(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    // Teleop
    public WizardBotDrive(OpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    // PID setup
    private void setup() {
        reduceTurn = true;
        isDrivePOV = true;
        isSquaredInputs = true;

        setPidTurn(p, i ,d);
        setPidAutoSpeed(0.028, 0.001, 0);
        setPidTurn(0.028, 0.001, 0);

        setMotorDir(false, true, false, true);
        setWheelDiameter(3.7795);
        setTicksPerRev(537.7);
    }

    public void updatePID() {setPidTurn(p, i, d);}

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
