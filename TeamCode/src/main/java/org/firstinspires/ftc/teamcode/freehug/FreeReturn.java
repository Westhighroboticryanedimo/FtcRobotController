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

    public boolean AiswithinBrangeofC(int a, int b, int c) {
        if(Math.abs(a-b)<=c) {
            return true;
        } else{
            return false;
        }
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
        if(AiswithinBrangeofC((int)(ang),10,90)) {
            xOffset += xChange;
            yOffset += yChange;
        } else if(AiswithinBrangeofC((int)(ang),10,0)) {
            xOffset += yChange;
            yOffset += xChange;
        } else if(AiswithinBrangeofC((int)(ang),10,180)) {
            xOffset -= yChange;
            yOffset += xChange;
        } else if(AiswithinBrangeofC((int)(ang),10,270)) {
            xOffset -= xChange;
            yOffset -= yChange;
        }
        //else, if is between 0 and 90,
        else if((ang > 0 && ang < 90)) {
            yOffset += yChange * Math.sin(radians_ang);
            xOffset += xChange * Math.cos(radians_ang);
        } else if((ang > 90 && ang < 180)) {
            yOffset += yChange * Math.cos(radians_ang - ninety_radians);
            xOffset += xChange * Math.cos(radians_ang - ninety_radians);
        } else if((ang > 180 && ang < 270)) {
            yOffset -= yChange * Math.sin(radians_ang - one_eighty_radians);
            xOffset -= xChange * Math.cos(radians_ang - one_eighty_radians);
        } else if((ang > 270 && ang <= 360)) {
            yOffset -= yChange * Math.cos(radians_ang - two_seventy_radians);
            xOffset -= xChange * Math.sin(radians_ang - two_seventy_radians);
        }
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