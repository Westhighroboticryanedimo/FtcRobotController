package org.firstinspires.ftc.teamcode.freehug;

import com.qualcomm.robotcore.hardware.DcMotor;

public class o_d_o_m_e_t_r_y implements Runnable{
    //public double COUNTS_PER_INCH = 307.699557;
    private double degrees_per_inch = 38.1971863421;
    private DcMotor vertical_1, vertical_2, horizontal;
    public boolean running = true;
    double vertical_1_position = 0, vertical_2_position = 0, horizontal_position = 0, change_in_angle = 0;
    public double robot_x = 0, robot_y = 0, robot_turn = 0;
    private double vertical_1_position_prev = 0, vertical_2_position_prev = 0, horizontal_position_prev = 0;
    private double encoder_wheel_distance, horizontal_tick_per_degree;
    private int sleep = 80;
    private int vertical_1_adjust = 1,vertical_2_adjust = 1,horizontal_adjust = 1;

    public void robot_position(DcMotor vertical_1, DcMotor vertical_2, DcMotor horizontal, double ticks_per_inch, int sleep) {
        this.vertical_1 = vertical_1;
        this.vertical_2 = vertical_2;
        this.horizontal = horizontal;
        this.sleep = sleep;
        encoder_wheel_distance = ticks_per_inch;
        horizontal_tick_per_degree = ticks_per_inch/degrees_per_inch;
        //robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
        //        this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
        //whatever that is
    }
    public void robot_position_update() {
        vertical_1_position = vertical_1.getCurrentPosition() * vertical_1_adjust;
        vertical_2_position = vertical_2.getCurrentPosition() * vertical_2_adjust;
        double v1_change = vertical_1_position - vertical_1_position_prev;
        double v2_change = vertical_2_position - vertical_2_position_prev;

        change_in_angle = (v1_change-v2_change) / encoder_wheel_distance;
        robot_turn = robot_turn + change_in_angle;

        horizontal_position = horizontal.getCurrentPosition() * horizontal_adjust;
        double raw_horizontal_change = horizontal_position - horizontal_position_prev;
        double horizontal_change = raw_horizontal_change - (change_in_angle*horizontal_tick_per_degree);

        double pp = ((v1_change - v2_change)/2);

        robot_x = (robot_x + pp*Math.sin(robot_turn) + horizontal_change*Math.cos(robot_turn));
        robot_y = (robot_y + pp*Math.cos(robot_turn) - horizontal_change * Math.sin(robot_turn));

        vertical_1_position_prev = vertical_1_position;
        vertical_2_position_prev = vertical_2_position;
        horizontal_position_prev = horizontal_position;
    }
    public double give_me_the_X() {return robot_x;}
    public double give_me_the_Y() {return robot_y;}
    public double give_me_the_ANGLE() {return robot_turn;}
    public void stop_it() {running = false;}
    public void recalibrate_position() {robot_x = 0; robot_y = 0; robot_turn = 0;}

    @Override
    public void run() {

        while(running) {

            robot_position_update();
            try {

                Thread.sleep(sleep);

            } catch (InterruptedException e) {

                e.printStackTrace();

            }
        }

    }
}

