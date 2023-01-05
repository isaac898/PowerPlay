package Winter_Autonomous_making;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;



public class Cycling_Tracking extends LinearOpMode {
    String Xposition;
    String Yposition;
    double overallTime;
    double time_left;
    double cycle_time;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime time = new ElapsedTime();
        time.startTime();  // returns the time at which the time has last reset
        time.reset(); // resets the time, and also displays the time that the timer was last reset
        time.time(TimeUnit.SECONDS); // returns the time from the last reset in SECONDS (I hope in seconds)

        // get the x and y position
        Xposition = "drivetrain.location.x";
        Yposition = "drivetrain.location.y";

        // overall time
        overallTime = getRuntime(); // returns the time in seconds
        telemetry.addData("Overall time: ", overallTime);
        // time left
        time_left = 30 - getRuntime(); // finds the left over time
        telemetry.addData("Time left", time_left); // checking to make sure that the time left is correct

        // next we move the robot to the position and then back
        cycle_time = overallTime; // this marks the time is will take to complete one cycle;

        while((time_left < cycle_time) && time_left > 3 ){
            // cycle
        }

        // park

    }
}

