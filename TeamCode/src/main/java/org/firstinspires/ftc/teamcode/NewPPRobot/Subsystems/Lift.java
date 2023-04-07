package org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems;
import org.firstinspires.ftc.teamcode.PIDController;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.abs;

public class Lift {
    private DcMotor lift1;
    private DcMotor lift2;
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
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void moveLift() {
        double encdrAvg = ((abs(lift1.getCurrentPosition())+abs(lift2.getCurrentPosition()))/2);
        pid.setSetpoint(liftTarget);
        double pidPower = pid.performPID(encdrAvg);
        if (pid.getSetpoint() == 0) {
            double power = pidPower;
        } else {
            double power = pidPower + ff;
        }
        lift1.setPower(power);
        lift2.setPower(-power);
    }
    public void setLiftPos(int tempTarget) {
        liftTarget = tempTarget;
        pid.reset();
        pid.enable();
        pid.reset();
    }

    //PID Tuning Methods
    public void changeLiftP(double change) {
        p = p + change;
    }
    public void changeLiftI(double change) {
        i = i + change;
    }
    public void changeLiftD(double change) {
        d = d + change;
    }
}
