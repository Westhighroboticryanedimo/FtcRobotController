package org.firstinspires.ftc.teamcode.freehug;

public class FreeReturn {
    public double xOffset;
    public double yOffset;
    public double robotCurrentAngle;
    //Freehugdrive drive;

    public void lockPosition() {
        xOffset = 0;
        yOffset = 0;
        robotCurrentAngle = 0;
    }

    public void updateOffsets(double xChange, double yChange) {
        double xChangeAdapted = xChange;
        double yChangeAdapted = yChange;
        double ang = robotCurrentAngle;

        //update x and y offset from original position, taking into account the direction it's facing
        if(ang >= 0 && ang <= 90) {
            //xOffset += xChange * cos(angle) ?? <-- something like this
            //yOffset += yChange *
        } else if(ang > 90 && ang <= 180) {
            //xOffset += xChange (relation to sin or cos or something
            //yOffset += yChange
        } else if(ang > 180 && ang <= 270) {
            //xOffset += xChange
            //yOffset += yChange
        } else if(ang > 270 && ang <= 360) {
            //xOffset += xChange
            //yOffset += yChange
        }

        xOffset += xChangeAdapted;
        yOffset += yChangeAdapted;
    }

    public void updateAngle(double angleChange) {
        robotCurrentAngle += angleChange;

        if(robotCurrentAngle > 360) {
            robotCurrentAngle = (robotCurrentAngle - 360);
        }
    }

    public void freelyReturn() {
        //move back to locked position
        //face forwards

    }
}