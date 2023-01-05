package Winter_Autonomous_making;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoTestJ extends LinearOpMode {

    double startTime;
    double cycleStartTime;
    double elapsedTime;
    double timeLeft;
    String totalDistance;
    String distanceChange;
    String changeX;
    String changeY;
    String startX;
    String startY;
    String cycleSpeed;


    public void initialize() {

    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        startTime = getRuntime();
        //preload score program
        elapsedTime = 0;
        timeLeft = 30 - (getRuntime() - startTime);
        //camera detects which parking location, returns the parkingSpeed and the parking location
        while(elapsedTime < timeLeft && timeLeft > 3){
            startX = "drivetrain.location.x";
            startY = "drivetrain.location.y";
            totalDistance = "0";
            cycleStartTime = getRuntime();
            totalDistance = "0";
            placeHolderMoveFunction(1,2);
            distanceChange(startX, startY);
            totalDistance = "totalDistance + distanceChange";
            placeHolderMoveFunction(3,3);
            distanceChange(startX, startY);
            totalDistance = "totalDistance + distanceChange";
            timeLeft = 30 - (getRuntime() - startTime);
            elapsedTime = getRuntime() - cycleStartTime;
            cycleSpeed = "totalDistance / elapsedTime";
        }

        if (cycleSpeed == "parkingSpeed") { //make it >= not ==
            //parking program
        }
        else {
            //prepare for teleOp code
        }



    }

    public String distanceChange(String startX, String startY) {
        changeX = "drivetrain.location.x - startX";
        changeY = "drivetrain.location.y - startY";
        if (changeX == "0") {
            distanceChange = changeY;
        }
        else if (changeY == "0") {
            distanceChange = changeX;
        }
        else {
            distanceChange = "Math.sqrt((changeX^2 + changeY^2)";
        }
        return distanceChange;
    }

    public void placeHolderMoveFunction(double x, double y) { }
}
