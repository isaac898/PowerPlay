package org.firstinspires.ftc.teamcode.manual_testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Imu;

@Autonomous
public class OdometryAndAutonomous extends LinearOpMode {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor br;
    private DcMotor bl;
    private BNO055IMU imu;



    @Override
    public void runOpMode() throws InterruptedException {

    }
}
