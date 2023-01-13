package org.firstinspires.ftc.teamcode.manual_testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class goStraight extends LinearOpMode{
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;


    @Override
    public void runOpMode() throws InterruptedException {

        //SET UP THE MOTORS FOR THE DRIVE TRAIN
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");

        //SET THE DIRECTIONS FOR THE MOTORS
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);


        goStraight();
        turnRight();
        turnLeft();

    }

    public void goStraight() {
        fl.setPower(0.4);
        bl.setPower(0.4);
        fr.setPower(0.4);
        br.setPower(0.4);

        sleep(2000); // run for one second

        killPower();

    }
    public void turnLeft(){
        fl.setPower(-0.7);
        bl.setPower(-0.7);
        fr.setPower(0.7);
        br.setPower(0.7);

        sleep(2000);

        killPower();
    }
    public void turnRight(){
        fl.setPower(0.7);
        bl.setPower(0.7);
        fr.setPower(-0.7);
        br.setPower(-0.7);

        sleep(2000);

        killPower();
    }
    public void killPower(){
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

}
