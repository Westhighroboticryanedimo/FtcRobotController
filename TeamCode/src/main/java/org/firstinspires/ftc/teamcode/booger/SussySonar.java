package org.firstinspires.ftc.teamcode.booger;

    public class SussySonar {
        /*
         *
    ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣴⣶⣿⣿⣷⣶⣄⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀
    ⠀⠀⠀⠀⠀⠀⠀⠀⠀⣰⣾⣿⣿⡿⢿⣿⣿⣿⣿⣿⣿⣿⣷⣦⡀⠀⠀⠀⠀⠀
    ⠀⠀⠀⠀⠀⠀⠀⢀⣾⣿⣿⡟⠁⣰⣿⣿⣿⡿⠿⠻⠿⣿⣿⣿⣿⣧⠀⠀⠀⠀
    ⠀⠀⠀⠀⠀⠀⠀⣾⣿⣿⠏⠀⣴⣿⣿⣿⠉⠀⠀⠀⠀⠀⠈⢻⣿⣿⣇⠀⠀⠀
    ⠀⠀⠀⠀⢀⣠⣼⣿⣿⡏⠀⢠⣿⣿⣿⠇⠀⠀⠀⠀⠀⠀⠀⠈⣿⣿⣿⡀⠀⠀
    ⠀⠀⠀⣰⣿⣿⣿⣿⣿⡇⠀⢸⣿⣿⣿⡀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⡇⠀⠀
    ⠀⠀⢰⣿⣿⡿⣿⣿⣿⡇⠀⠘⣿⣿⣿⣧⠀⠀⠀⠀⠀⠀⢀⣸⣿⣿⣿⠁⠀⠀
    ⠀⠀⣿⣿⣿⠁⣿⣿⣿⡇⠀⠀⠻⣿⣿⣿⣷⣶⣶⣶⣶⣶⣿⣿⣿⣿⠃⠀⠀⠀
    ⠀⢰⣿⣿⡇⠀⣿⣿⣿⠀⠀⠀⠀⠈⠻⣿⣿⣿⣿⣿⣿⣿⣿⣿⠟⠁⠀⠀⠀⠀
    ⠀⢸⣿⣿⡇⠀⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠉⠛⠛⠛⠉⢉⣿⣿⠀⠀⠀⠀⠀⠀
    ⠀⢸⣿⣿⣇⠀⣿⣿⣿⠀⠀⠀⠀⠀⢀⣤⣤⣤⡀⠀⠀⢸⣿⣿⣿⣷⣦⠀⠀⠀
    ⠀⠀⢻⣿⣿⣶⣿⣿⣿⠀⠀⠀⠀⠀⠈⠻⣿⣿⣿⣦⡀⠀⠉⠉⠻⣿⣿⡇⠀⠀
    ⠀⠀⠀⠛⠿⣿⣿⣿⣿⣷⣤⡀⠀⠀⠀⠀⠈⠹⣿⣿⣇⣀⠀⣠⣾⣿⣿⡇⠀⠀
    ⠀⠀⠀⠀⠀⠀⠀⠹⣿⣿⣿⣿⣦⣤⣤⣤⣤⣾⣿⣿⣿⣿⣿⣿⣿⣿⡟⠀⠀⠀
    ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠻⢿⣿⣿⣿⣿⣿⣿⠿⠋⠉⠛⠋⠉⠉⠁⠀⠀⠀⠀
    ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠉⠉⠁
        eres muy sospechoso, mi amigo.
         */
        public double botx, boty, botrotation; // rotation acts like a unit circle. start out pointing 'up' which would be 90 degrees.
        public double fieldwidth, fieldheight;
        double leftdist, rightdist, frontdist, backdist;
        double leftdistrotation, rightdistrotation, frontdistrotation, backdistrotation; // the directions the sensors are facing
        String side; // which side of the field do we start on

        public void setup() {botx = fieldwidth/2; boty = fieldheight/2; botrotation = 90;}

        public void denoteFieldDimensions(double fw, double fh) {
            fieldwidth = fw; fieldheight = fh;
        }

        public double[] getCoords() {return new double[] {botx,boty};}

        public double x() {return botx;} public double y() {return boty;}

        public void update(double baserotation, double fd, double rd, double bd, double ld) {
            botrotation = baserotation;
            updateSensorDirections(baserotation);
            assessPositionFromSensorInputs(fd, rd, bd, ld);
            //System.out.println("rotation: " + rotation + " leftdist: " + leftdist + " rightdist: " + rightdist + " frontdist: " + frontdist + " backdist: " + backdist + " leftdistrotation: " + leftdistrotation);
        }

        public void updateSensorDirections(double baserotation) { // keep track of where the sensors are pointed
            frontdistrotation = baserotation;
            rightdistrotation = rotateAngle(baserotation,-90);
            backdistrotation = rotateAngle(baserotation,180);
            leftdistrotation = rotateAngle(baserotation,90);
        }
        public double[] imaginarySensorDirections(double baserotation) {
            return new double[] {baserotation, rotateAngle(baserotation,-90),rotateAngle(baserotation,180),rotateAngle(baserotation,90)};
            // front, right, back, left
        }

        public double rotateAngle(double anglestart, double angleadjustment) { // adjust angle by given degrees
            //System.out.println("angle " + anglestart + " adjusted by " + angleadjustment + " to become:");
            anglestart += angleadjustment;

            if(anglestart > 360) {
                while(anglestart > 360) {anglestart -= 360;}
            } else if(anglestart < 0) {
                while(anglestart < 0) {anglestart += 360;}
            }
            //System.out.println(anglestart + " <--");
            return anglestart;
        }

        public void assessPositionFromSensorInputs(double fd, double rd, double bd, double ld) { // take in sensor data
            leftdist = ld; rightdist = rd; frontdist = fd; backdist = bd;
            // given four beams, their lengths and the angles they came from, find the x and y value at which the robot sits.
            // create an imaginary robot with the same rotation, start in the middle of the field, and adjust location until its imaginary sensors read the same values.
            double ix, iy, ileftdist, irightdist, ifrontdist, ibackdist; // imaginary values
            ix = fieldwidth/2; iy = fieldheight/2;
            int steps = 0;
        /*
        : FAILED ALGORITHM :
        double at = 3; // accuracy threshold
        double[] isv = imaginarySensorValues(botrotation, ix, iy); // imaginary sensor values
        ifrontdist = isv[0]; irightdist = isv[1]; ibackdist = isv[2]; ileftdist = isv[3];

        double[] rightd = cartesianFromPolar(1, rotateAngle(botrotation,-90)); // 'right' direction represented as x and y components
        double[] forwardd = cartesianFromPolar(1, botrotation); // 'rorward' direction represented as x and y components
        //System.out.println("finding position, " + ileftdist + " left dist "  + irightdist + " <- rightdist right x: " + rightd[0] + " right y: " + rightd[1]);
        //System.out.println("front x: " + forwardd[0] + " front y: " + forwardd[1]);
        int steps = 0;
        while(!closeEnough(ld,ileftdist,at) || !closeEnough(rd,irightdist,at) || !closeEnough(fd,ifrontdist,at) || !closeEnough(bd,ibackdist,at)) {
            /*
            // move imaginary bot around until imaginary values match real values.
            if(Math.floor(steps%50)%2 == 0) {
                if(irightdist > rd || ileftdist < ld) {// if imaginary right value is larger than it should be (meaning too far left)
                    ix += rightd[0]; iy += rightd[1]; // move right, leading to smaller distance to wall
                    //System.out.println("moving right");
                } else if(irightdist < rd || ileftdist > ld) { // if imaginary right value is too small
                    ix -= rightd[0]; iy -= rightd[1];
                    //System.out.println("moving left");
                }
                isv = imaginarySensorValues(botrotation, ix, iy); // imaginary sensor values
            } else {
                if(ifrontdist > fd || ibackdist < bd) {// if front value is larger than it should be (meaning too far back)
                    ix += forwardd[0]; iy += forwardd[1]; // move forward, leading to smaller distance
                    //System.out.println("moving forward");
                } else if(ifrontdist < fd || ibackdist > bd) {
                    ix -= forwardd[0]; iy -= forwardd[1];
                    //System.out.println("moving back");
                }
                isv = imaginarySensorValues(botrotation, ix, iy); // imaginary sensor values
            }
            ifrontdist = isv[0]; irightdist = isv[1]; ibackdist = isv[2]; ileftdist = isv[3];
            //if(steps%11==0) {System.out.println("coords: " + ix + " , " + iy + " rightdist: " + irightdist + " frontdist: " + ifrontdist);}
            steps++;
            if(!inField(ix,iy)) {break;}
            if(steps > 1000) break;
            //if(steps%12==0) System.out.println("desired values: " + rd + " , " + fd + " , " + ld + " , " + fd + "|| current values: " + irightdist + " , " + ifrontdist + " , " + ileftdist + " , " + ifrontdist);
        }
        : FAILED ALGORITHM :
        */
            // new method: consider the 4 ends of the sensor lines as points, and move these points to fit within the box.
            double[][] spoints = new double[][] {cartesianFromPolar(fd, frontdistrotation),cartesianFromPolar(rd, rightdistrotation),cartesianFromPolar(bd, backdistrotation),cartesianFromPolar(ld, leftdistrotation)};
            // ix and iy start out in the middle of the points.
        /*double[] midpointoffsetfromleftsensorpoint = cartesianFromPolar(ld,imaginarySensorDirections(botrotation)[3]); // the offset from the left sensor point to the robot point.
        ix = spoints[3][0]-midpointoffsetfromleftsensorpoint[0];
        iy = spoints[3][1]-midpointoffsetfromleftsensorpoint[1];*/
            // put ix and iy in the center of all the sensor points.
            ix = spoints[0][0] + ((spoints[2][0]-spoints[0][0])*(fd/(fd+bd)));
            iy = spoints[1][1] + ((spoints[3][1]-spoints[1][1])*(rd/(rd+ld)));
            while(!inField(spoints[0][0],spoints[0][1],10) || !inField(spoints[1][0],spoints[1][1],10) || !inField(spoints[2][0],spoints[2][1],10) || !inField(spoints[3][0],spoints[3][1],10) || !inField(ix,iy,10)) {
                for(int i = 0; i < spoints.length; i++) {
                    if(spoints[i][0] < -5) {
                        ix += Math.abs(spoints[i][0]);
                        System.out.println("x moved by " + Math.abs(spoints[i][0]));
                        spoints = transformAllPointsInArray(spoints, Math.abs(spoints[i][0]), 0);
                    } else if(spoints[i][0] > fieldwidth+5) {
                        ix -= Math.abs(fieldwidth-spoints[i][0]);
                        System.out.println("x moved by " + -Math.abs(fieldwidth-spoints[i][0]));
                        spoints = transformAllPointsInArray(spoints, -Math.abs(fieldwidth-spoints[i][0]),0);
                    }
                    if(spoints[i][1] < -5) {
                        iy += Math.abs(spoints[i][1]);
                        spoints = transformAllPointsInArray(spoints, 0, Math.abs(spoints[i][1]));
                    } else if(spoints[i][1] > fieldheight+5) {
                        iy -= Math.abs(fieldheight-spoints[i][1]);
                        spoints = transformAllPointsInArray(spoints, 0, -Math.abs(fieldheight-spoints[i][1]));
                    }
                }
                steps++;
                if(steps > 444) {break;}
                System.out.println(ix + " , " + iy);
                //SonarTester.print(spoints[0][0] + " , " + spoints[0][1] + " | " + spoints[1][0] + " , " + spoints[1][1] + " | " + spoints[2][0] + " , " + spoints[2][1] + " | " + spoints[3][0] + " , " + spoints[3][1] + " | " + " :: " + ix + " , " + iy);
            }
            // though adjustments have been made such that all points are within the field, errors are still possible because
            // sometimes there are gaps between points and the wall, which cannot physically happen on a blank field.
            // because points are shifted in from the left, they will always be flush with the left wall at first.
            // it is only necessary to shift rightward (and up, since we're also coming in from the bottom).
            while(!closeEnoughToWall(spoints[0][0],spoints[0][1],10) || !closeEnoughToWall(spoints[1][0],spoints[1][1],10) || !closeEnoughToWall(spoints[2][0],spoints[2][1],10) || !closeEnoughToWall(spoints[3][0],spoints[3][1],10)) {
                // shifting right until right wall is reached
                if(inField(spoints[0][0]+5,spoints[0][1],10) && inField(spoints[1][0]+5,spoints[1][1],10) && inField(spoints[2][0]+5,spoints[2][1],10) && inField(spoints[3][0],spoints[3][1]+5,10)) {
                    ix += 5;
                    spoints = transformAllPointsInArray(spoints, 5, 0);
                    //System.out.println("moving x: " + ix);
                } else {break;}
                steps++;
                if(steps > 444) {break;}
            }
            while(!closeEnoughToWall(spoints[0][0],spoints[0][1],10) || !closeEnoughToWall(spoints[1][0],spoints[1][1],10) || !closeEnoughToWall(spoints[2][0],spoints[2][1],10) || !closeEnoughToWall(spoints[3][0],spoints[3][1],10)) {
                // shifting up until top wall is reached
                if(inField(spoints[0][0],spoints[0][1]+5,10) && inField(spoints[1][0],spoints[1][1]+5,10) && inField(spoints[2][0],spoints[2][1]+5,10) && inField(spoints[3][0],spoints[3][1]+5,10)) {
                    iy += 5;
                    spoints = transformAllPointsInArray(spoints, 0, 5);
                    //System.out.println("moving y: " + iy);
                } else {break;}
                steps++;
                if(steps > 2000) {break;}
            }
            //System.out.println(botrotation + " --> left+right: " + (ileftdist + irightdist) + " front+back: " + (ifrontdist + ibackdist));
            botx = ix; boty = iy;
        }

        public double[][] transformAllPointsInArray(double[][] array, double xdisp, double ydisp) {
            for(int i = 0; i < array.length; i++) {
                array[i][0] += xdisp;
                array[i][1] += ydisp;
            }
            return array;
        }

        public boolean closeEnough(double a, double b, double accuracy) {
            return Math.abs(a-b) <= accuracy;
        }

        public boolean closeEnoughToWall(double x, double y, double a) { // a for accuracy
            return (
                    closeEnough(x,0,a) || closeEnough(x,fieldwidth,a) || closeEnough(y,0,a) || closeEnough(y,fieldheight,a)
            );
        }

        public double[] imaginarySensorValues(double rotation, double x, double y) { // what would the sensors read at this rotation and these coordinates
            double[] directions = imaginarySensorDirections(rotation);
            double[] response = new double[] {lengthUntilWall(x,y,directions[0]),lengthUntilWall(x,y,directions[1]),lengthUntilWall(x,y,directions[2]),lengthUntilWall(x,y,directions[3])};
            return response;
        }

        public double distance(double x, double y, double xx, double yy) {
            return Math.sqrt(Math.pow(x-xx,2) + Math.pow(y-yy,2));
        }

        public boolean inField(double x, double y) {
            return (x >= 0 && y >= 0 && x <= fieldwidth && y <= fieldheight);
        }

        public boolean inField(double x, double y, double accuracy) {
            return (x + accuracy >= 0 && y + accuracy >= 0 && x - accuracy <= fieldwidth && y - accuracy <= fieldheight);
        }

        public double lengthUntilWall(double x, double y, double rotation) { // x, y, x displacement, y displacement
            double length = 0;
            double maxlength = 2*distance(0,0,fieldwidth,fieldheight); // the maximum length a beam could travel: the distance between far corners of the field.
            boolean hitwall = false;

            // input was a direction, so get the x and y components of a vector length 1 in that direction.
            double[] cart = cartesianFromPolar(0.2,rotation);

            double xd = cart[0]; double yd = cart[1]; // these are the components.
            //System.out.println(x + " , " + y + " , " + xd + " , " + yd + " , " + maxlength);
            double cx = x; double cy = y; // checking x, checking y
            while(length < maxlength && !hitwall) { // to check the distance until the wall, shift test values in that direction until they are no longer within the bounds of the field and record the distance they have travelled.
                if(!inField(cx,cy)) {hitwall = true;}
                cx += xd;
                cy += yd;
                //System.out.println(cx + " , " + cy);
            }
            //System.out.println("distance to wall in direction " + rotation + " from " + x + " , " + y + ": " + length);
            //if(rotation >= 40 && rotation <= 50) {System.out.println(rotation + ": " + length);}
            length = distance(x,y,cx,cy);
            return length;
        }

        // mug mug
        static double[] cartesianFromPolar(double r, double theta) {
            theta = (theta / 180) * Math.PI;
            return new double[]{r * Math.cos(theta), r * Math.sin(theta)};
        }
        public static double[] polarFromCartesian(double x, double y) {
            //return new double[]{Math.sqrt(x * x + y * y), Math.atan2(y, x)}; //red
            return new double[]{Math.sqrt(x * x + y * y), (Math.atan2(y, x) * 180) / Math.PI}; //cart
        }
        // mug mug
        public int roundDouble(double in) {
            int up = (int)Math.ceil(in);
            int down = (int)Math.floor(in);
            if(Math.abs(in-up) < Math.abs(in-down)) {return up;} else {return down;}
        }

        public char[][] getMapImage(double x, double y, double rotation, int mapWidth) {
            char[][] mapImage = new char[mapWidth][mapWidth];
            double fieldscale = fieldwidth / mapImage.length;

            double[] sensors = imaginarySensorDirections(rotation);
            // vectors of sensor directions
            double[][] sensorCartesians = new double[][]{cartesianFromPolar(1, sensors[0]), cartesianFromPolar(1, sensors[1]), cartesianFromPolar(1, sensors[2]), cartesianFromPolar(1, sensors[3])};
            // sensor exacts: where the sensors allegedly collided with the wall, relative to bot location.
            double[][] sensorExacts = new double[][]{cartesianFromPolar(frontdist, sensors[0]), cartesianFromPolar(rightdist, sensors[1]), cartesianFromPolar(backdist, sensors[2]), cartesianFromPolar(leftdist, sensors[3])};
            for (int mx = 0; mx < mapImage.length; mx++) {
                for (int my = mapImage[0].length - 1; my >= 0; my--) {
                    if (closeEnough(mx * fieldscale, botx, 10 + (fieldwidth / (fieldscale))) && closeEnough(my * fieldscale, boty, 16 + (fieldheight / fieldscale))) {

                        // draw sensor path lines
                        for (int i = 0; i < sensorCartesians.length; i++) {
                            boolean donedrawing = false;
                            double dx = botx;
                            double dy = boty;
                            while (!donedrawing) {
                                if (roundDouble(dx) <= 0 || roundDouble(dx) >= (mapImage.length - 1) * fieldscale || roundDouble(dy) <= 0 || roundDouble(dy) >= (mapImage[0].length - 1) * fieldscale) {
                                    donedrawing = true;
                                    break;
                                }
                                dx += sensorCartesians[i][0];
                                dy += sensorCartesians[i][1];
                                mapImage[roundDouble(dx / fieldscale)][mapImage[0].length - 1 - roundDouble(dy / fieldscale)] = '.';
                            }
                        }
                    }
                    // draw map edge
                    else if (mx * fieldscale <= fieldscale - 1 || my * fieldscale <= fieldscale - 1 || mx * fieldscale >= (mapImage.length * fieldscale) - (fieldscale) || my * fieldscale >= (mapImage[0].length * fieldscale) - (fieldscale)) {
                        if (closeEnough(mx * fieldscale, botx, 40 + (fieldwidth / fieldscale)) || closeEnough(my * fieldscale, boty, 40 + (fieldheight / fieldscale))) {
                            mapImage[mx][mapImage[0].length - 1 - my] = '■';
                        } else {
                            mapImage[mx][mapImage[0].length - 1 - my] = '▢';
                        }
                    } else {
                        if (mapImage[mx][mapImage[0].length - 1 - my] != '.')
                            mapImage[mx][mapImage[0].length - 1 - my] = ' ';
                    }
                }
            }

            // draw sensor's path end points
            for(int i = 0; i < sensorExacts.length; i++) {
                int sx = roundDouble(roundDouble((botx+sensorExacts[i][0]))/fieldscale);
                int sy = mapImage[0].length - 1- roundDouble(roundDouble((boty+sensorExacts[i][1]))/fieldscale);
                if(sx <= 0) {sx = 1;} else if(sx >= mapImage.length) {sx = mapImage.length-2;}
                if(sy <= 0) {sy = 1;} else if(sy >= mapImage.length) {sy = mapImage[0].length-2;}
                mapImage[sx][sy] = 'X';
            }
            int bx = roundDouble(botx/fieldscale);
            int by = roundDouble(boty/fieldscale);
            if(bx <= 0) {bx = 1;} else if(bx >= mapImage.length) {bx = mapImage.length-2;}
            if(by <= 0) {by = 1;} else if(by >= mapImage.length) {by = mapImage[0].length-2;}
            // draw the robot itself
            mapImage[bx][mapImage[0].length-1-by] = '骨';
            return mapImage;
        }
        public void printMapImage(char[][] mapImage) {
            System.out.println("==================");
            System.out.println("at " + botx + " , " + boty + "   rotation --> " + botrotation);
            for(int my = 0; my < mapImage[0].length; my++) {
                String mapLine = "";
                for(int mx = 0; mx < mapImage.length; mx++) {
                    mapLine = mapLine + mapImage[mx][my] + " ";//mapImage[mx][my];
                }
                System.out.println(mapLine + " |" + my);
            }
        }

    }
