package testingMechanisms;

import android.content.CursorLoader;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="LiftTest")
public class PowerPlayTestForLift extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    // VARIABLES FOR THE LIFT
    private DcMotor rightLiftMotor;
    private DcMotor leftLiftMotor;
    boolean flag = true;
    private int position = 0;


    @Override
    public void init(){
       telemetry.addData("Status", "Initialized");

       // set up the motors
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        leftLiftMotor = hardwareMap.get(DcMotor.class, "llMotor");

        // set the direction both in the same way
        rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // set the motors to run with encoder
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set target to zero
        rightLiftMotor.setTargetPosition(0);
        leftLiftMotor.setTargetPosition(0);

        // stop and reset the encoders
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the mode to run to position
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightLiftMotor.setPower(0.9);
        leftLiftMotor.setPower(0.9);

        telemetry.update();


   }

    @Override
    public void loop() {
        if(gamepad1.x){
            if(flag){
                position += 50;
                rightLiftMotor.setTargetPosition(position);
                leftLiftMotor.setTargetPosition(position);

                flag = false;
            }
        } else if (gamepad1.b){
            if(flag){
                position -= 50;
                rightLiftMotor.setTargetPosition(position);
                leftLiftMotor.setTargetPosition(position);

                flag = false;
            }
        } else {
            flag = true;
        }
        if(gamepad1.a){
            rightLiftMotor.setTargetPosition(100);
            leftLiftMotor.setTargetPosition(100);
        }
        if(gamepad1.dpad_down){
            rightLiftMotor.setTargetPosition(0);
            leftLiftMotor.setTargetPosition(0);
        }
        if(gamepad1.dpad_right){
            rightLiftMotor.setTargetPosition(500);
            leftLiftMotor.setTargetPosition(500);
        }
        if(gamepad1.dpad_left){
            rightLiftMotor.setTargetPosition(1000);
            leftLiftMotor.setTargetPosition(1000);
        }


        telemetry.addData("Motors ", "right_position: ", rightLiftMotor.getCurrentPosition());
        telemetry.addData("Motors ", "left_position:  ", leftLiftMotor.getCurrentPosition());
        telemetry.update();

    }


}