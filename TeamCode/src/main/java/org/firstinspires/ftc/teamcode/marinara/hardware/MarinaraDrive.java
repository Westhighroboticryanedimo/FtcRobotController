package org.firstinspires.ftc.teamcode.marinara.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.drive.HolonomicDrive;

public class MarinaraDrive extends HolonomicDrive {

    private ColorSensor color;
    private DistanceSensor distanceRight;
    private DistanceSensor distanceBack;
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
        setWheelDiameter(3.9);
        setTicksPerRev(537.6);

        // Color sensor on bot
        color = hwMap.get(ColorSensor.class, "color");

        // Distance sensors on bot
        distanceBack = hwMap.get(DistanceSensor.class, "distanceBack");
        distanceRight = hwMap.get(DistanceSensor.class, "distanceRight");

    }

    public int[] getColors() {

        return new int[] { color.red(), color.green(), color.blue() };

    }

    public double getDistanceBack() {

        return distanceBack.getDistance(DistanceUnit.INCH);

    }

    public double getDistanceRight() {

        return distanceRight.getDistance(DistanceUnit.INCH);

    }

    // Check color sensor status
    public class ColorCommand implements BoolCommand {

        @Override
        public boolean check() {

            return getColors()[2] > BLUE_THRESHOLD;

        }

    }

    // Check if right distance is over a certain threshold
    public class DistanceRightCommand implements BoolCommand {

        private final double distance;

        public DistanceRightCommand(double distance) {

            this.distance = distance;

        }

        @Override
        public boolean check() {

            return distance > getDistanceRight();

        }

    }

    // Check if back distance is over a certain threshold
    public class DistanceBackCommand implements BoolCommand {

        private final double distance;

        public DistanceBackCommand(double distance) {

            this.distance = distance;

        }

        @Override
        public boolean check() {

            return distance > getDistanceBack();

        }

    }

}
