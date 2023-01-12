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

@TeleOp (name= "Draft2PowerPlay")
public class PowerPlayMechanisms extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // VARIABLES FOR THE CLAW
    private Servo claw_servo;
    private boolean open = true;

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

    private int  rightLiftPosition;
    private int leftLiftPosition;
    private int liftIncrement = 100;
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
        claw_servo.setPosition(0);
        telemetry.addData("Motors", "right (%.2f)", claw_servo.getPosition());

        //SET UP THE MOTORS FOR THE DRIVE TRAIN
        fl = hardwareMap.get(DcMotor.class, "FL");
        fr = hardwareMap.get(DcMotor.class, "FR");
        bl = hardwareMap.get(DcMotor.class, "BL");
        br = hardwareMap.get(DcMotor.class, "BR");

        //SET THE DIRECTIONS FOR THE MOTORS
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // set up the two lift motors
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        leftLiftMotor = hardwareMap.get(DcMotor.class, "llMotor");

        // set up the two arm servos
        right_arm = hardwareMap.get(Servo.class, "rServo");
        left_arm = hardwareMap.get(Servo.class, "lServo");



        // set the arm position to the correct positions
        right_arm.setPosition(0.75);
        left_arm.setPosition(0.25);


        // set the direction of the motors
        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // set the motors to run with encoder
        rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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



        // CODE FOR THE CLAW
        if (gamepad2.b) {
            if(open){
                claw_servo.setPosition(0.65); // claw is open
                open = false;
            } else {
                open = true;
            }

            claw_servo.setPosition(0.4); // claw is closed

        }

        // CODE FOR THE LIFT
        // 3.138 : 1 ratio old to new junction position
        // set up the lift motors to move to the set positions at the same time
        if (gamepad2.dpad_up){ // high junction
            rightLiftMotor.setTargetPosition(1625);
            leftLiftMotor.setTargetPosition(1625);
        } else if(gamepad2.dpad_left){
            rightLiftMotor.setTargetPosition(825);
            leftLiftMotor.setTargetPosition(824);
        } else if(gamepad2.dpad_down){
            rightLiftMotor.setTargetPosition(0);
            leftLiftMotor.setTargetPosition(0);
        }

//        if (gamepad2.left_stick_button){ // going up
//            if(flag){
//                rArmPosition = right_arm.getPosition();
//                lArmPosition = left_arm.getPosition();
//                right_arm.setPosition(rArmPosition -= increment);
//                left_arm.setPosition(lArmPosition += increment);
//                flag = false;
//            }
//
//        } else if (gamepad2.right_stick_button) {
//            if (flag) {
//                rArmPosition = right_arm.getPosition();
//                lArmPosition = left_arm.getPosition();
//                right_arm.setPosition(rArmPosition+=increment);
//                left_arm.setPosition(lArmPosition-=increment);
//                flag = false;
//            }
//        } else {
//            flag = true;
//        }
        // TOGGLE THE LIFT CODE
        if(gamepad2.left_trigger > 0.05){ // lift goes up
            if(flag){
                rightLiftPosition = rightLiftMotor.getCurrentPosition();
                leftLiftPosition = leftLiftMotor.getCurrentPosition();
                rightLiftMotor.setTargetPosition(rightLiftPosition += liftIncrement);
                leftLiftMotor.setTargetPosition(leftLiftPosition += liftIncrement);
                flag = false;

            }
        } else if (gamepad2.right_trigger > 0.05){ // lift goes down
            if (flag) {
                rightLiftPosition = rightLiftMotor.getCurrentPosition();
                leftLiftPosition = leftLiftMotor.getCurrentPosition();
                rightLiftMotor.setTargetPosition(rightLiftPosition -= liftIncrement); // this says --> rightliftposition = rightliftposition  - liftincrement
                leftLiftMotor.setTargetPosition(leftLiftPosition -= liftIncrement);
                flag = false;// 626-848-1207
            }

        } else {
            flag = true;
        }


        // CODE FOR LIFTING AND DROPPING THE ARMS
        if (gamepad2.y){ // go high
            right_arm.setPosition(0.45);
            left_arm.setPosition(0.55);

        }
        if (gamepad2.x) { // go mid
            right_arm.setPosition(0.65); // closer to one, means lower
            left_arm.setPosition(0.35); // closer to zero, means lower
        }

        if (gamepad2.a){ // go low
            right_arm.setPosition(0.95);
            left_arm.setPosition(0.05);
        }


        if (gamepad1.x){
            position += 100;
            rightLiftMotor.setTargetPosition(position);
            leftLiftMotor.setTargetPosition(position);
        }
        if (gamepad1.b) {
            position -= 100;
            rightLiftMotor.setTargetPosition(position);
            leftLiftMotor.setTargetPosition(position);
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