package org.firstinspires.ftc.teamcode.Odometry;

import java.lang.Math;

public class OdometryTesting01 {

    // variables for the left encoder
    double delta_left_encoder_pos;
    double left_encoder_pos;
    double prev_left_encoder_pos;
    // variables for the right encoder
    double delta_right_encoder_pos;
    double right_encoder_pos;
    double prev_right_encoder_pos;
    // variables for the center encoder
    double delta_center_encoder_pos;
    double center_encoder_pos;
    double prev_center_encoder_pos;
    // phi
    double phi;
    //trackwidth
    double trackwidth;
    // change in middle pos
    double delta_middle_pos;
    // change in perp pos
    double delta_perp_pos;
    // forward offset
    double forward_offset;
    //change in x
    double delta_x;
    // change in y
    double delta_y;
    // x position
    double x_pos;
    // y position
    double y_pos;
    // heading
    double heading;




    public void odometry() {

        delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
        delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
        delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos;

        phi = (delta_left_encoder_pos - delta_right_encoder_pos) / trackwidth;
        delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;
        delta_perp_pos = delta_center_encoder_pos - forward_offset * phi;

        delta_x = delta_middle_pos * Math.cos(heading) - delta_perp_pos * Math.sin(heading);
        delta_y = delta_middle_pos * Math.sin(heading) + delta_perp_pos * Math.cos(heading);

        x_pos += delta_x;
        y_pos += delta_y;
        heading += phi;

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;
        prev_center_encoder_pos = center_encoder_pos;

    }
}







