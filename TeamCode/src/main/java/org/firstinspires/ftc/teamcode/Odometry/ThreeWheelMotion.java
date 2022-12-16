package org.firstinspires.ftc.teamcode.Odometry;


public class ThreeWheelMotion {
    // VARIABLES
    double yChange = 0.0;
    double xChange = 0.0;
    double degChange = 0.0;

    // INITIAL ENCODER POSITIONS
    int lastLeftEncoder = 0;
    int lastRightEncoder = 0;
    int last_a_encoder = 0; // i assume this is going to be the center encoder

    // used for reading angle absolutely not integrated
    double angleRadBias = 0.0; // not sure what this does

    double lastRawAngle = 0.0; // not sure what this does



    public void update(int curr_l_encoder, int curr_r_encoder, int curr_a_encoder, int leftInchesPerTick, double rightInchesPerTick, double auxInchesPerTick, double turnTrackWidth, double auxTrackWidth) {
        double DeltaWheel_left = (curr_l_encoder - lastLeftEncoder ) * leftInchesPerTick; // not sure why we multiply by that value
        double DeltaWheel_right = (curr_r_encoder - lastRightEncoder) * rightInchesPerTick; // not sure why we multiple by that value
        double DeltaWheel_a = (curr_a_encoder - last_a_encoder) * auxInchesPerTick; // not sure why we multiple by that value

        // calculates angle change for running arc integration and aux prediction
        double angleIncrement = (DeltaWheel_right- DeltaWheel_left) / turnTrackWidth;

        // use absolute for actual angle
        double leftTotal = curr_l_encoder * rightInchesPerTick; // not sure why we multiiply by that value
        double rightTotal = curr_r_encoder * rightInchesPerTick; // not sure why we multiple by that value
        double lastRawAngle = ( (leftTotal - rightTotal) / turnTrackWidth );
        double finalAngleRad = lastRawAngle + angleRadBias;

        // the aux wheel moves when we rotate, so cancel this out with a prediciton
        double aux_prediction = angleIncrement * auxTrackWidth;
        // not sure why we include a prediction

        double yDelta = (DeltaWheel_left + DeltaWheel_right) / 2;
        double xDelta = DeltaWheel_a - aux_prediction;

        //create a function named update position
        // we need to find a way to update the position
        // we are using the values xDelta, yDelta, angleIncrement

        // update the current encoder parts to the last encoder part

        //Vector2d myVector = new Vector2d(10, -5);






    }

}
