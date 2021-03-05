package org.firstinspires.ftc.teamcode.freehug;

public class FreeReturn {
    public double xOffset;
    public double yOffset;
    public double robotCurrentAngle;
    public boolean freely_hugging = false;
    //Freehugdrive drive;

    public void lockPosition() {
        xOffset = 0;
        yOffset = 0;
        robotCurrentAngle = 0;
    }

    public void updateOffsets(double xChange, double yChange) {
        double ang = robotCurrentAngle;
        int radians_ang = (int) (robotCurrentAngle * Math.PI/180);

        //update x and y offset, taking into account the direction it's facing
        yOffset += yChange * Math.sin(radians_ang);
        xOffset += xChange * Math.cos(radians_ang);
    }

    public void updateAngle(double angleChange) {
        robotCurrentAngle -= angleChange;

        if(robotCurrentAngle > 360) {
            robotCurrentAngle = (robotCurrentAngle - 360);
        } else if(robotCurrentAngle < 0) {
            robotCurrentAngle = (robotCurrentAngle + 360);
        }
    }
}