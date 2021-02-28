package org.firstinspires.ftc.teamcode.marinara.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class MarinaraDrive extends HolonomicDrive {

    private ColorSensor color;

    // For teleop
    public MarinaraDrive(OpMode opMode, HardwareMap hwMap) {

        super(opMode, hwMap);
        setup(hwMap);

    }

    // For autonomous
    public MarinaraDrive(LinearOpMode opMode, HardwareMap hwMap) {

        super(opMode, hwMap);
        setup(hwMap);

    }

    // Setup
    private void setup(HardwareMap hwMap) {

        // PID Values
        setPidDrive(0.05, 0, 0);
        setPidSpeed(0.05, 0.001, 0);
        setPidTurn(0.03, 0.001, 0);

        // Robot characteristics
        setWheelDiameter(4);
        setTicksPerRev(1120);
        // Color sensor on bot
        color = hwMap.get(ColorSensor.class, "color");

    }

    // Check color sensor status
    public class ColorCommand implements BoolCommand {

        @Override
        public boolean check() {

            return color.blue() > color.red();

        }

    }

}
