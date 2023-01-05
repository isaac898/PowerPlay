package Winter_Autonomous_making;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class CycleTill5Left extends LinearOpMode{
    // get the time
   // double elapsedTime;

    // get the time left
    double timeLeft;

    @Override
    public void runOpMode() throws InterruptedException {
       // elapsedTime = getRuntime(); // get the overall run time
        timeLeft = 30 - getRuntime();

        // cycle while the time left is greater than 3 seconds
        while(timeLeft > 3) {
            PlaceHolderMoveFunction(1,3);
        }

    }

    public void PlaceHolderMoveFunction(double X, double Y) { }
}