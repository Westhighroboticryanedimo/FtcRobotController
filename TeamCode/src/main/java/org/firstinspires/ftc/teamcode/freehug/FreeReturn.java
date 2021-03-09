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
        //if angle is 0, x += yChange , y -= xChange
        //if angle is 90 , y and x are unaffected
        //if angle is 180,y += xChange, x -= yChange
        //if angle is 270, x += -xChange , y += -yChange
        /*if(ang == 90) {
            xOffset += xChange;
            yOffset += yChange;
        }*/
        xOffset += xChange;
        yOffset += yChange;
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