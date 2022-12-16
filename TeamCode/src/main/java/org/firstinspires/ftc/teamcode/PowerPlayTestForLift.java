package org.firstinspires.ftc.teamcode;

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
    private DcMotor rightliftMotor;
    private DcMotor leftLiftMotor;
    boolean flag = true;


    @Override
    public void init(){
       telemetry.addData("Status", "Initialized");

        rightliftMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        leftLiftMotor = hardwareMap.get(DcMotor.class, "llMotor");

        rightliftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.update();



   }

    @Override
    public void loop() {

        // goes up
       if(gamepad2.dpad_up){
               rightliftMotor.setPower(0.2);
               leftLiftMotor.setPower(0.2);
       } else {
           rightliftMotor.setPower(0);
           leftLiftMotor.setPower(0);
       }

       // goes down
       if(gamepad2.dpad_down){
           rightliftMotor.setPower(-0.2);
           leftLiftMotor.setPower(-0.2);
       } else {
           rightliftMotor.setPower(0);
           leftLiftMotor.setPower(0);
       }


       if(gamepad2.dpad_left){
           rightliftMotor.setPower(0);
           leftLiftMotor.setPower(0);
       }


       telemetry.addData("Status", "Rune Time: " + runtime.toString() );
       telemetry.addData("Power:", "Right Power: " + rightliftMotor.getPower() );
       telemetry.addData("Power:", "Left Power: " + leftLiftMotor.getPower() );



        telemetry.update();

    }


}