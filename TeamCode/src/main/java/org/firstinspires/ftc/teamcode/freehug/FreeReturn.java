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
        robotCurrentAngle = 90;
    }

    public void updateOffsets(double xChange, double yChange) {
        double ang = robotCurrentAngle;
        int radians_ang = (int) (robotCurrentAngle * (Math.PI/180));
        int ninety_radians = (int) (90 * (Math.PI/180));
        int one_eighty_radians = (int) (Math.PI);
        int two_seventy_radians = ninety_radians + one_eighty_radians;
        int three_sixty_radians = one_eighty_radians * 2;

        //update x and y offset, taking into account the direction it's facing
        //yOffset += yChange * Math.cos(radians_ang);
        //xOffset += xChange * Math.sin(radians_ang);
        if(ang > 0 && ang <= 90) {
            yOffset += yChange * Math.sin(radians_ang);
            xOffset += xChange * Math.cos(radians_ang);
        } else if(ang > 90 && ang <= 180) {
            yOffset += yChange * Math.cos(radians_ang - ninety_radians);
            xOffset += xChange * Math.sin(radians_ang - ninety_radians);
        } else if(ang > 180 && ang <= 270) {
            yOffset += yChange * Math.sin(radians_ang - one_eighty_radians);
            xOffset += xChange * Math.cos(radians_ang - one_eighty_radians);
        } else if(ang > 270 && ang <= 360) {
            yOffset += yChange * Math.cos(radians_ang - two_seventy_radians);
            xOffset += xChange * Math.sin(radians_ang - two_seventy_radians);
        }
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