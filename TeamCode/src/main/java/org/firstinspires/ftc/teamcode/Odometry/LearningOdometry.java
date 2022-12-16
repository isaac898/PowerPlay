package org.firstinspires.ftc.teamcode.Odometry;

import android.content.CursorLoader;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.*;

import java.util.*;


@Autonomous(name = "OdometryTesting")
public class LearningOdometry extends LinearOpMode {

    class Values{
        public void main(String arg[]){
            // tuned (taken from Calvin, with consent)
            double TARGET_LOCATION_TOLERANCE_INCHES = 2.5;
            double TARGET_HEADING_TOLERANCE_DEGREES = 3.0;

            double X_ODOMETRY_COUNTS_PER_ROTATION = 45004.1666;
            double Y_ODOMETRY_COUNTS_PER_ROTATION = 61288.8295;
            double DEGREES_PER_ROTATION = 360.0;

            double TRANSLATIONAL_DECELERATION_INCHES_PER_SECOND_PER_SECOND = 50.0;
            double ROTATIONAL_DECELERATION_DEGREES_PER_SECOND_PER_SECOND = 515.0;
            double TRANSLATIONAL_DECELERATION_MULTIPLIER = 0.7;
            double ROTATION_DECELERATION_MULTIPLIER = 1.0;

            // measured (taken from Calvin, with consent)
            double ODOMETRY_WHEEL_COUNTS_PER_ROTATION = 8192.0;
            double ODOMETRY_WHEEL_DIAMETER_MILLIMETERS = 35.0;
            double MILLIMETERS_PER_INCH = 25.4;
            double PI = 3.141592653589;

            // derived
            double ODOMETRY_WHEEL_DIAMETER_INCHES = ODOMETRY_WHEEL_DIAMETER_MILLIMETERS / MILLIMETERS_PER_INCH;

            double ODOMETRY_WHEEL_INCHES_PER_ROTATION = ODOMETRY_WHEEL_DIAMETER_INCHES * PI;

            double ODOMETRY_WHEEL_COUNTS_PER_INCH = ODOMETRY_WHEEL_COUNTS_PER_ROTATION / ODOMETRY_WHEEL_INCHES_PER_ROTATION;

            double X_ODOMETRY_COUNTS_PER_DEGREE = X_ODOMETRY_COUNTS_PER_ROTATION / DEGREES_PER_ROTATION;

            double Y_ODOMETRY_COUNTS_PER_DEGREE = Y_ODOMETRY_COUNTS_PER_ROTATION / DEGREES_PER_ROTATION;

        }
    }

/////////////////////////////// //////////////////////// /////////////////// ///////////////// ///////////////////

    // CONFIGURE THE HUB?

    // CONFIGURING THE MOTORS
    DcMotorEx fr = null;
    DcMotorEx fl = null;
    DcMotorEx br = null;
    DcMotorEx bl = null;


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
