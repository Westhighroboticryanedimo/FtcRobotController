
package org.firstinspires.ftc.teamcode.slowBolon;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name = "REDTONOMOUS+++++++++++++++++++++++++++++a")
public class REDtonomous extends LinearOpMode {

    //private org.firstinspires.ftc.teamcode.slowBolon.CamBolon Cam;
    private int DONDEESTAELDUCKY;
    private OdometryBolon o;
    private DriveBolon d;

    double distance;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        //Cam = new org.firstinspires.ftc.teamcode.slowBolon.CamBolon();
        //Cam.init(hardwareMap);

        OdometryBolon o = new OdometryBolon();
        o.init(hardwareMap.get(DcMotor.class,"frontLeft"));
        hardwareMap.get(DcMotor.class,"frontLeft").setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareMap.get(DcMotor.class,"frontLeft").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        distance = o.distance;

        o.run();
        d = new DriveBolon(this,hardwareMap);
        //DONDEESTAELDUCKY = Cam.getspot();
        ElapsedTime runtime = new ElapsedTime();

        //telemetry.addData("where",DONDEESTAELDUCKY);
        /*telemetry.addData("leastduckydiff", Cam.pipeline.leastduckydiff);
        telemetry.addData("leasttapediff", Cam.pipeline.leasttapediff);
        telemetry.addData("x", Cam.pipeline.gx);
        telemetry.addData("y", Cam.pipeline.gy);
        telemetry.addData("gYELLOW", Cam.pipeline.greatestyellow);
        telemetry.addData("distance",o.distance);
        telemetry.addData("HELLOW",1);
        telemetry.update();

        int dx = Cam.pipeline.gx;
        int wthird = (int)(Math.floor(Cam.pipeline.w/3));


        if(dx <= wthird) {
            while(distance < 400) {
                d.drive(-0.4,0,0);
                o.updatedistance();
                distance=o.distance;
                telemetry.addData("WHERE","LEFT");
                telemetry.update();
            }
        }

        else if(dx <= 2*wthird) {
            while(distance < 400) {
                d.drive(0,0.4,0);
                o.updatedistance();
                distance=o.distance;
                telemetry.addData("WHERE","MIDDLE");
                telemetry.update();
            }
        }

        else {
            while(distance < 400) {
                d.drive(0.4,0,0);
                o.updatedistance();
                distance=o.distance;
                telemetry.addData("WHERE","RIGHT");
                telemetry.update();
            }
        }*/

        while(distance < 1100) {
            d.drive(0,-0.4,0);
            o.updatedistance();
            distance=o.distance;
            telemetry.addData("distahcee",o.distance);
            telemetry.update();
        }
        o.resetdistance(); distance=o.distance;
        while(distance < 1000) {
            d.drive(-0.4,0,0);
            o.updatedistance();
            distance=o.distance;
            telemetry.addData("dostance",o.distance);
            telemetry.update();
        }
    }
}
