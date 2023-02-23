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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@TeleOp(name = "Draft2PowerPlay")

public class PowerPlayMechanisms extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // toggling the arm
    private double leftPosition;
    private double rightPosition;

    // toggle the lift
    private int rPosition;
    private int lPosition;

    // testing for the lift
    private boolean flag2;

    // VARIABLES FOR THE CLAW
    private Servo claw_servo;

    // VARIABLES FOR THE FOUR MOTORS
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;

    // VARIABLES FOR THE LIFT (ALWAYS INT)
    private DcMotor rightLiftMotor;
    private DcMotor leftLiftMotor;
    private Servo right_arm;
    private Servo left_arm;

    boolean flag = true;

    private int position = 0;
//    private double rArmPosition; <-- irrelevant
//    private double lArmPosition; <-- irrelevant
//    private double aIncrement = 0.05; <-- irrelevant


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //SET UP THE CLAW
        claw_servo = hardwareMap.get(Servo.class, "cServo");
        claw_servo.setDirection(Servo.Direction.REVERSE);
        telemetry.addData("Motors", "right (%.2f)", claw_servo.getPosition());

        //SET UP THE MOTORS FOR THE DRIVE TRAIN
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        //SET THE DIRECTIONS FOR THE MOTORS
        fl.setDirection(DcMotorSimple.Direction.REVERSE); // port 0
        fr.setDirection(DcMotorSimple.Direction.FORWARD); // port 1
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD); // port 2

//        // set the motors to break
//        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set up the two lift motors
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        leftLiftMotor = hardwareMap.get(DcMotor.class, "llMotor");

        // set up the two arm servos
        right_arm = hardwareMap.get(Servo.class, "rServo");
        left_arm = hardwareMap.get(Servo.class, "lServo");

        //set the positions for the arms to the down position
        right_arm.setPosition(1);
        left_arm.setPosition(0);

        // set the direction of the motors
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

        // stop and reset the encoders
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightLiftMotor.setPower(0.9);
        leftLiftMotor.setPower(0.9);

        telemetry.update();
    }

    @Override
    public void loop() {

        // SETTING THE VARIABLES FOR THE FOUR MOTORS
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        double drive_y = -gamepad1.left_stick_y;
        double drive_x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x; // random comment

//        if(gamepad1.right_trigger == 0){
//            frontLeftPower =   Range.clip(drive_y + drive_x + turn, -0.5, 0.5);
//            frontRightPower = Range.clip(drive_y - drive_x - turn, -0.5, 0.5);
//            backLeftPower =  Range.clip(drive_y - drive_x + turn, -0.5,0.5);
//            backRightPower =   Range.clip(drive_y + drive_x - turn, -0.5, 0.5);
//
//        } else {
//            frontLeftPower =  Range.clip(drive_y + drive_x + turn, -0.7, 0.7);
//            frontRightPower =  Range.clip(drive_y - drive_x - turn, -0.7, 0.7);
//            backLeftPower =  Range.clip(drive_y - drive_x + turn, -0.7,0.7);
//            backRightPower =  Range.clip(drive_y + drive_x - turn, -0.7, 0.7);
//        }
        frontLeftPower = drive_y + drive_x + turn;
        frontRightPower = drive_y - drive_x - turn;
        backLeftPower = drive_y - drive_x + turn;
        backRightPower = drive_y + drive_x - turn;

        // store the values in a list of doubles
        List<Double> list = Arrays.asList(1.0, Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower));

        // get the greatest value
        double maximum = Collections.max(list); // returns the greatest number


        // set the powers
        fl.setPower((frontLeftPower / maximum) * 0.6);
        if(fl.isBusy()){fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}

        fr.setPower((frontRightPower / maximum) * 0.6);
        if(fr.isBusy()){fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}

        bl.setPower((backLeftPower / maximum) * 0.6);
        if(bl.isBusy()){bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}

        br.setPower((backRightPower / maximum) * 0.6);
        if(br.isBusy()){br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);} else {br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}

        if(gamepad1.right_trigger > 0.01){
            fl.setPower((frontLeftPower / maximum) * 0.8);
            fr.setPower((frontRightPower / maximum) * 0.8);
            bl.setPower((backLeftPower / maximum) * 0.8);
            br.setPower((backRightPower / maximum) * 0.8);
        }

        if(gamepad1.right_bumper){
            fl.setPower((frontLeftPower / maximum) * 0.2);
            fr.setPower((frontRightPower / maximum) * 0.2);
            bl.setPower((backLeftPower / maximum) * 0.2);
            br.setPower((backRightPower / maximum) * 0.2);
        }


        // CODE FOR THE CLAW // works
        if (gamepad1.left_trigger != 0) { // open the claw
            claw_servo.setPosition(0.2);
        } else {
            claw_servo.setPosition(0.43); // close the claw
        }

        if (gamepad2.right_bumper) { // close the claw
            claw_servo.setPosition(0.43);
        }

        if(gamepad1.x){
            right_arm.setPosition(1); // bring the arm back down
            left_arm.setPosition(0);

            rightLiftMotor.setTargetPosition(0); // bring the lift down
            leftLiftMotor.setTargetPosition(0);

            if(rightLiftMotor.getCurrentPosition() == 0){
                claw_servo.setPosition(0.2);
            } else if (leftLiftMotor.getCurrentPosition() == 0){
                claw_servo.setPosition(0.2);
            } else {
                claw_servo.setPosition(0.2);
            }


        }

        // CODE FOR THE LIFT
        // 3.138 : 1 ratio old to new junction position
        // set up the lift motors to move to the set positions at the same time
        //works
        if (gamepad2.dpad_up) { // high junction
            rightLiftMotor.setTargetPosition(1610);
            leftLiftMotor.setTargetPosition(1610);

        }

        if (gamepad2.dpad_left) { // middle junction
            rightLiftMotor.setTargetPosition(1083);
            leftLiftMotor.setTargetPosition(1083);


        }

        if(gamepad2.dpad_right){
            rightLiftMotor.setTargetPosition(434);
            leftLiftMotor.setTargetPosition(434);
        }

        if (gamepad2.dpad_down) { // go low
            rightLiftMotor.setTargetPosition(0);
           leftLiftMotor.setTargetPosition(0);

        }



//        // testing the is busy method
//        while(rightLiftMotor.isBusy()){
//
//        }


        // CODE FOR LIFTING AND DROPPING THE ARMS // works
        if (gamepad2.y) { // go high
            right_arm.setPosition(0.1); // 0.25
            left_arm.setPosition(0.9); // 0.75
        }
//        if (gamepad2.x) { // go mid
//            right_arm.setPosition(0.65); // closer to one, means lower
//            left_arm.setPosition(0.35); // closer to zero, means lower
        // to make the push push
//        }

        if (gamepad2.a) { // go low
            right_arm.setPosition(1);
            left_arm.setPosition(0);
        }

        // toggle the arms
        if (gamepad2.left_bumper) {
            if (flag) {
                if ( ( (right_arm.getPosition() - 0.05) > 0.05) && ( (left_arm.getPosition() + 0.05) < 0.95) ) {
                    leftPosition = left_arm.getPosition() + 0.05;
                    rightPosition = right_arm.getPosition() - 0.05;
                }

                right_arm.setPosition(rightPosition);
                left_arm.setPosition(leftPosition);

                flag = false;
            }
        } else if (gamepad2.b) {
            if (flag) {
                leftPosition = left_arm.getPosition() - 0.05;
                rightPosition = right_arm.getPosition() + 0.05;

                right_arm.setPosition(rightPosition);
                left_arm.setPosition(leftPosition);

                flag = false;
            }
        } else {
            flag = true;
        }

// toggle the lift // works
        if (gamepad2.left_trigger > 0.01) {
            if (flag2) {
                rPosition = rightLiftMotor.getCurrentPosition() - 75;
                lPosition = leftLiftMotor.getCurrentPosition() - 75;

                rightLiftMotor.setTargetPosition(rPosition);
                leftLiftMotor.setTargetPosition(lPosition);

                flag2 = false;
            }

        } else if (gamepad2.right_trigger > 0.01) {
            if (flag2) {
                if( ( (rightLiftMotor.getCurrentPosition() + 75) < 1610) && ( (leftLiftMotor.getCurrentPosition() + 75) < 1610) ) {
                    rPosition = rightLiftMotor.getCurrentPosition() + 75;
                    lPosition = leftLiftMotor.getCurrentPosition() + 75;

                    rightLiftMotor.setTargetPosition(rPosition);
                    leftLiftMotor.setTargetPosition(lPosition);
                }
                flag2 = false;
            }

        } else {
            flag2 = true;
        }

        // set the right arm to 0.6 for the low junction
        // set the right from 1 to 0.6

        // the left is going to go from zero to 0.4


        telemetry.addData("Status", "Rune Time: " + runtime.toString());
        telemetry.addData("Encoder count Right: ", +rightLiftMotor.getCurrentPosition());
        telemetry.addData("Encoder count left: ", + leftLiftMotor.getCurrentPosition());
        telemetry.addData("Absolute encoder right: ", + Math.abs(rightLiftMotor.getCurrentPosition()));
        // tests to see when the power
//        telemetry.addData("Encoder Counts Left: ", +leftLiftMotor.getCurrentPosition()2);
        telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Gamepad coordinates", "(x,y) = (%.2f, %.2f), turn = %.2f", drive_x, drive_y, turn);
        telemetry.addData("Servo Position", "Claw position: (%5.2f)", claw_servo.getPosition());
        telemetry.addData("Servo", "right arm:  (%.2f)", right_arm.getPosition());
        telemetry.addData("Servo", "left arm: (%.2f)", left_arm.getPosition());


        telemetry.update();


    }
    public final void waitUntil(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}