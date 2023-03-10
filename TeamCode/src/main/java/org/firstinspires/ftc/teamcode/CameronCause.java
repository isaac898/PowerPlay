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

@TeleOp (name= "CameronCode")
public class CameronCause extends OpMode {

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

        rightLiftMotor.setPower(0.8);
        leftLiftMotor.setPower(0.8);

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

        if(gamepad1.right_trigger == 0){
            frontLeftPower =   Range.clip(drive_y + drive_x + turn, -0.5, 0.5);
            frontRightPower = Range.clip(drive_y - drive_x - turn, -0.5, 0.5);
            backLeftPower =  Range.clip(drive_y - drive_x + turn, -0.5,0.5);
            backRightPower =   Range.clip(drive_y + drive_x - turn, -0.5, 0.5);

        } else {
            frontLeftPower =  Range.clip(drive_y + drive_x + turn, -0.7, 0.7);
            frontRightPower =  Range.clip(drive_y - drive_x - turn, -0.7, 0.7);
            backLeftPower =  Range.clip(drive_y - drive_x + turn, -0.7,0.7);
            backRightPower =  Range.clip(drive_y + drive_x - turn, -0.7, 0.7);
        }

        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);


        // CODE FOR THE CLAW // works
        if (gamepad2.right_bumper) { // open the claw
            claw_servo.setPosition(0.5);
        }
        if(gamepad2.left_bumper) { // close the claw
            claw_servo.setPosition(0.2);
        }



        // CODE FOR THE LIFT
        // 3.138 : 1 ratio old to new junction position
        // set up the lift motors to move to the set positions at the same time
        //works
        if (gamepad2.dpad_up){ // high junction
            rightLiftMotor.setTargetPosition(1143);
            leftLiftMotor.setTargetPosition(1143);
        }

        if(gamepad2.dpad_left){ // middle junction
            rightLiftMotor.setTargetPosition(251);
            leftLiftMotor.setTargetPosition(251);
        }

        if(gamepad2.dpad_down){ // go low
            rightLiftMotor.setTargetPosition(0);
            leftLiftMotor.setTargetPosition(0);
        }


        // CODE FOR LIFTING AND DROPPING THE ARMS // works
        if (gamepad2.y){ // go high
            right_arm.setPosition(0.2); // 0.25
            left_arm.setPosition(0.8); // 0.75
        }
        if (gamepad2.x) { // go mid
            right_arm.setPosition(0.6); // closer to one, means lower
            left_arm.setPosition(0.4); // closer to zero, means lower
        }

        if (gamepad2.a){ // go low
            right_arm.setPosition(1);
            left_arm.setPosition(0);
        }

        // toggle the arms
        if(gamepad2.dpad_right){
            if(flag) {
                if((right_arm.getPosition() > 0.45) && (left_arm.getPosition() < 0.55)) {
                    leftPosition = left_arm.getPosition() + 0.05;
                    rightPosition = right_arm.getPosition() - 0.05;
                }

                right_arm.setPosition(rightPosition);
                left_arm.setPosition(leftPosition);

                flag = false;
            }
        } else if (gamepad2.b){
            if(flag){
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
            if(flag2){
                if(((rightLiftMotor.getCurrentPosition() + 75) < 1000) && ((leftLiftMotor.getCurrentPosition() + 75) < 1000) ){
                    rPosition = rightLiftMotor.getCurrentPosition() + 75;
                    lPosition = leftLiftMotor.getCurrentPosition() + 75;
                    rightLiftMotor.setTargetPosition(rPosition);
                    leftLiftMotor.setTargetPosition(lPosition);
                    flag2 = false;
                }
            }

        } else {
            flag2 = true;
        }

        // set the right arm to 0.6 for the low junction
        // set the right from 1 to 0.6

        // the left is going to go from zero to 0.4





        telemetry.addData("Status", "Rune Time: " + runtime.toString() );
        telemetry.addData("Encoder count Right: ", + rightLiftMotor.getCurrentPosition());
        telemetry.addData("Encoder Counts Left: ", + leftLiftMotor.getCurrentPosition());
        telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f)", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Gamepad coordinates","(x,y) = (%.2f, %.2f), turn = %.2f", drive_x, drive_y, turn);
        telemetry.addData("Servo Position", "Claw position: (%5.2f)" , claw_servo.getPosition());
        telemetry.addData("Servo", "right arm:  (%.2f)", right_arm.getPosition());
        telemetry.addData("Servo", "left arm: (%.2f)", left_arm.getPosition());



        telemetry.update();



    }
}