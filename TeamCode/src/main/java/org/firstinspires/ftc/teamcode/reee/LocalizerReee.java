package org.firstinspires.ftc.teamcode.reee;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.roadrunner.drive.Drive;

import org.firstinspires.ftc.teamcode.utils.localization.PointMapLoc;

public class LocalizerReee extends PointMapLoc {
    public LocalizerReee(LinearOpMode opMode, HardwareMap hwMap, Drive drive) {
        super(opMode, hwMap, drive);
        setup();
    }

    public LocalizerReee(OpMode opMode, HardwareMap hwMap, Drive drive) {
        super(opMode, hwMap, drive);
        setup();
    }

    private void setup() {
        distFromCenter = 3;
    }
}
