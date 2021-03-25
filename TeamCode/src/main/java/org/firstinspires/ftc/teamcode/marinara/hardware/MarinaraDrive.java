package org.firstinspires.ftc.teamcode.marinara.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class MarinaraDrive extends HolonomicDrive {

    private ColorSensor color;
    private static final int BLUE_THRESHOLD = 100;

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
        setPidDrive(0.03, 0, 0);
        setPidSpeed(0.03, 0.0008, 0);
        setPidTurn(0.03, 0.0008, 0);

        // Robot characteristics
        setWheelDiameter(4);
        setTicksPerRev(560);
        // Color sensor on bot
        color = hwMap.get(ColorSensor.class, "color");

    }

    public int[] getColors() {

        return new int[] { color.red(), color.green(), color.blue() };

    }

    // Check color sensor status
    public class ColorCommand implements BoolCommand {

        @Override
        public boolean check() {

            return getColors()[2] > BLUE_THRESHOLD;

        }

    }

}
