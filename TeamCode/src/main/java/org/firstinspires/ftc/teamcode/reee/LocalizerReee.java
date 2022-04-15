package org.firstinspires.ftc.teamcode.reee;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.localization.PointMapLoc;

public class LocalizerReee extends PointMapLoc {
    public LocalizerReee(LinearOpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    public LocalizerReee(OpMode opMode, HardwareMap hwMap) {
        super(opMode, hwMap);
        setup();
    }

    private void setup() {
        ticksPerRev = 103.8;
        distFromCenter = 4;
    }
}
