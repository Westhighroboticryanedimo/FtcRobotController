package org.firstinspires.ftc.teamcode.NewPPRobot.Subsystems;
import org.firstinspires.ftc.teamcode.PIDController;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

public class ChodeLift {
    private DcMotor lift1;
    private DcMotor lift2;
    private TouchSensor liftLimit;

    public PIDController pid = new PIDController(p, i, d);
    public static double p =0.006 ;
    public static double i = 0;
    public static double d = 0.005;
    public static double ff = 0.2;

    double finalPower = 0;
    double pidPower = 0;
    double encdrAvg = 0;

    public void liftInit(HardwareMap hardwareMap) {
        liftLimit = hardwareMap.get(TouchSensor.class, "liftLimit");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        pid.reset();
        pid.setInputRange(0, 4000);
        pid.setOutputRange(0.0, 1.0);
        pid.setTolerance(0);
        pid.enable();

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveLift() {
        pid.setPID(p, i, d);
        double encdrAvg = (abs(lift1.getCurrentPosition())+abs(lift2.getCurrentPosition()))/2;
        double pidPower = pid.performPID(encdrAvg);
        double finalPower = pidPower + ff;
        lift1.setPower(finalPower);
        lift2.setPower(-finalPower);
    }

    public void setLiftPos(double liftTarget) {
        pid.setSetpoint(liftTarget);
        pid.reset();
        pid.enable();
    }

    public int arrived() {
        if (abs(((lift1.getCurrentPosition() + -1 * lift2.getCurrentPosition())/2) - pid.getSetpoint())<60) {
            return(1);
        } else {return(0);}
    }

    //PID Tuning Methods
    public void changeLiftP(double change) {p = p + change;}
    public void changeLiftI(double change) {i = i + change;}
    public void changeLiftD(double change) {d = d + change;}
    public void changeLiftFF(double change) {ff = ff + change;}
    public double getPIDPower() {return(pidPower);}
    public double getP() {return(p);}
    public double getI() {return(i);}
    public double getD() {return(d);}
    public double getFF() {return(ff);}
    public double getSetpoint() {return(pid.getSetpoint());}
    public double getEncoderAverage() {return(encdrAvg);}
}
