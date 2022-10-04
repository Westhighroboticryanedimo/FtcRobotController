package org.firstinspires.ftc.teamcode.booger;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Gyro;

@TeleOp(name = "booger teleop")
public class BoogerTeleOp extends OpMode {

    private SussySonar sonar;
    private ModernRoboticsI2cRangeSensor f;
    private ModernRoboticsI2cRangeSensor r;
    private ModernRoboticsI2cRangeSensor b;
    private ModernRoboticsI2cRangeSensor l;
    //private Gyro gyro;
    private static double inchestosensoredge;
    private static double gyrooffset;


    @Override
    public void init() {
        inchestosensoredge = 2; // distance between edge of sensor and midpoint, to be subtracted from measurement
        gyrooffset = 0; // make sure 90 degrees means 'forwards'
        sonar = new SussySonar();
        sonar.setup();
        sonar.denoteFieldDimensions(720,720);
        f = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontsensor");
        r = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rightsensor");
        b = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "backsensor");
        l = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "leftsensor");

        //gyro = new Gyro(hardwareMap,false);
        //gyro.reset();
    }
    @Override
    public void loop() {
        telemetry.addData("running?","yes");
        double frontdist = f.getDistance(DistanceUnit.INCH)+inchestosensoredge;
        double rightdist = r.getDistance(DistanceUnit.INCH)+inchestosensoredge;
        double leftdist = l.getDistance(DistanceUnit.INCH)+inchestosensoredge;
        double backdist = b.getDistance(DistanceUnit.INCH)+inchestosensoredge;
        //sonar.update(gyro.getAngleDegrees(),frontdist*10,rightdist*10,backdist*10,leftdist*10);

        //sonar.update(90,frontdist*10,rightdist*10,backdist*10,leftdist*10);

        sonar.update(70,frontdist*10,rightdist*10,sonar.fieldheight-(frontdist*10),sonar.fieldwidth-(rightdist*10));

        //char[][] mapImage = sonar.getMapImage(sonar.botx,sonar.boty,gyro.getAngleDegrees(),14); // minimap
        char[][] mapImage = sonar.getMapImage(sonar.botx,sonar.boty,90,14); // minimap

        for(int my = 0; my < mapImage[0].length; my++) { // add minimap to telemetry
            String mapLine = "";
            for(int mx = 0; mx < mapImage.length; mx++) {
                mapLine = mapLine + mapImage[mx][my] + "     ";//mapImage[mx][my];
            }
            telemetry.addData("|", mapLine + " " + my);
        }
        telemetry.addData("x: " , sonar.x());
        telemetry.addData("y: " , sonar.y());
        telemetry.addData("front: " , f.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }
}
