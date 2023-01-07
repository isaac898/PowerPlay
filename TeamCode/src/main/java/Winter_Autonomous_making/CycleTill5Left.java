package Winter_Autonomous_making;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class CycleTill5Left extends LinearOpMode{
    // get the time
   // double elapsedTime;


    // get the time left
    double timeLeft;
    DcMotor rightLiftMotor;
    DcMotor leftLiftMotor;
    Servo servoClaw;



    @Override
    public void runOpMode() throws InterruptedException {
        // set up the claw and the lift
        setUp();

       // elapsedTime = getRuntime(); // get the overall run time
        timeLeft = 30 - getRuntime();

        // cycle while the time left is greater than 3 seconds
        while(timeLeft > 3) {
            cycle();
            //checkTime();

        }

    }

    public void PlaceHolderMoveFunction(double X, double Y) { }

    public void LiftxAmount(int target ) {
        rightLiftMotor.setTargetPosition(target);
        leftLiftMotor.setTargetPosition(target);


        rightLiftMotor.setPower(0.5);
        leftLiftMotor.setPower(0.5);

        while (leftLiftMotor.isBusy() && rightLiftMotor.isBusy()) {

        }

        killPower();
    }

    public void killPower() {
        rightLiftMotor.setPower(0);
        leftLiftMotor.setPower(0);
    }

    public void Grap(){
        servoClaw.setPosition(0.7);
    }

    public void Open(){
        servoClaw.setPosition(0);
    }

    public void cycle(){
        PlaceHolderMoveFunction(0,3); // move up
        PlaceHolderMoveFunction(-3,3); // move to the left
        // it turns
        LiftxAmount(1000); // replace 1000 with a variable that changes when grabbing the high, middle, and low cones
        Grap();
        PlaceHolderMoveFunction(0,3); // move to the right
        // it turns and scores on the high junction
        LiftxAmount(1625);
        Open();
        // it turns
    }

    public void setUp(){
        // CODE FOR THE CLAW
        servoClaw = hardwareMap.get(Servo.class, "cServo");
        servoClaw.setDirection(Servo.Direction.REVERSE);
        servoClaw.setPosition(0);

        // SET UP THE VARIABLES FOR THE LIFT
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        leftLiftMotor = hardwareMap.get(DcMotor.class, "llMotor");

        // set to run using encoder
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set target position to zero
        rightLiftMotor.setTargetPosition(0);
        leftLiftMotor.setTargetPosition(0);

        // stop and reset the encoders
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // stop and reset the encoders
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}