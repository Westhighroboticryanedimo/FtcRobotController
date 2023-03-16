package org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems;
import org.firstinspires.ftc.teamcode.PIDController;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.abs;

public class Lift {
    private DcMotor liftMotor;
    private DcMotor liftMotor2;
    private TouchSensor liftLimit;

    public PIDController pid = new PIDController(p, i, d);
    public static double p =0 ;
    public static double i = 0;
    public static double d = 0;
    public static double ff = 0;

    public static int liftTarget = 0;
    double power = 0;

    public void liftInit(HardwareMap hardwareMap) {
        liftLimit = hardwareMap.get(TouchSensor.class, "liftLimit");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotor.class, "liftMotor2");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveLift() {
        double encdrAvg = ((abs(liftMotor.getCurrentPosition())+abs(liftMotor2.getCurrentPosition()))/2);
        pid.setSetpoint(liftTarget);
        double pidPower = pid.performPID(encdrAvg);
        if (pid.getSetpoint() == 0) {
            double power = pidPower;
        } else {
            double power = pidPower + ff;
        }
        liftMotor.setPower(power);
        liftMotor2.setPower(-power);
    }
    public void setLiftPos(int tempTarget) {
        liftTarget = tempTarget;
        pid.reset();
        pid.enable();
        pid.reset();
    }

    //PID Tuning Methods
    public void changeP(double change) {
        p = p + change;
    }
    public void changeI(double change) {
        i = i + change;
    }
    public void changeD(double change) {
        d = d + change;
    }
}
