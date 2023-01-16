package org.firstinspires.ftc.teamcode.manual_testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class goStraight extends LinearOpMode{
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private Servo cServo;
    private Servo rightArm;
    private Servo leftArm;
    private DcMotor rLift;
    private DcMotor lLift;


    @Override
    public void runOpMode() throws InterruptedException {
        // set up the claw
        cServo = hardwareMap.get(Servo.class, "cServo");
        cServo.setDirection(Servo.Direction.REVERSE);

        // set up the arms
        rightArm = hardwareMap.get(Servo.class, "rServo");
        leftArm = hardwareMap.get(Servo.class, "lServo");

        //SET UP THE MOTORS FOR THE DRIVE TRAIN
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        // set up the lift motors
        rLift = hardwareMap.get(DcMotor.class, "rlMotor");
        lLift = hardwareMap.get(DcMotor.class, "llMotor");

        //SET THE DIRECTIONS FOR THE MOTORS
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // set the lift direction
       setUpLift();

        waitForStart();


//        closeClaw();
//        sleep(1000);
//        liftArm();
//        liftMotors();
//        goStraight();
//        turnRight();
//        openClaw();



    }

    public void goStraight() {
        fl.setPower(0.4);
        bl.setPower(0.4);
        fr.setPower(0.4);
        br.setPower(0.4);

        sleep(1800);

        killPower();

    }
    public void turnLeft(){
        fl.setPower(-0.7);
        bl.setPower(-0.7);
        fr.setPower(0.7);
        br.setPower(0.7);

        sleep(1000);

        killPower();
    }
    public void turnRight(){
        fl.setPower(0.7);
        bl.setPower(0.7);
        fr.setPower(-0.7);
        br.setPower(-0.7);

        sleep(200);

        killPower();
    }
    public void killPower(){
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
    public void closeClaw(){
        cServo.setPosition(0.2);
    }
    public void openClaw(){
        cServo.setPosition(0.5);
    }
    public void liftArm(){
        rightArm.setPosition(0.6);
        leftArm.setPosition(0.4);
    }
    public void liftMotors(){
        rLift.setTargetPosition(1000);
        lLift.setTargetPosition(1000);
    }
    public void setUpLift(){
        rLift.setDirection(DcMotorSimple.Direction.FORWARD);
        lLift.setDirection(DcMotorSimple.Direction.FORWARD);

        // set the motors to run with encoder
        rLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // set target to zero
        rLift.setTargetPosition(0);
        lLift.setTargetPosition(0);

        // stop and reset the encoders
        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // stop and reset the encoders
        rLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rLift.setPower(0.8);
        lLift.setPower(0.8);
    }
}
