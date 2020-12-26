package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ExampleHardware {

    // Motor attributes
    private DcMotor motorLeft;
    private DcMotor motorRight;

    public ExampleHardware(HardwareMap hwMap) {

        // Assign motors; device name is configured in Driver Station
        motorLeft = hwMap.get(DcMotor.class, "motorLeft");
        motorRight = hwMap.get(DcMotor.class, "motorRight");

        // Set the direction of the motors
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        // Set power to 0
        motorLeft.setPower(0);
        motorRight.setPower(0);

    }

    public void drive(double joystickY, double joystickTurn) {

        /*
        Joystick Y is reversed in FTC... idk why (positive when down, negative when up)
        Joystick X is normal so no need to fix (positive when right, negative when left)
        */
        joystickY *= -1;

        /*
        Think conceptually, when you move your left joystick up (positive value) and your right joystick
        right (positive value), which motor should move forward (be positive) and which should move backward
        (be negative)?
         */
        double leftPower = joystickY + joystickTurn;
        double rightPower = joystickY - joystickTurn;

        /*
        Now what if joystickY was 0.9 and joystickTurn was 0.7? Well, 0.9 + 0.7 = 1.6 and 0.9 - 0.7 = 0.2.
        So, leftPower = 1.6 and rightPower = 0.2. See the problem? setPower() can't have a number greater than
        1 in its parameter! So, to fix this, you have to divide both by the bigger number. Math.max() returns
        the greater of the two numbers
         */
        double max = Math.max(leftPower, rightPower);
        leftPower /= max;
        rightPower /= max;

        // Finally, the the power of the motors
        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);

    }

}
