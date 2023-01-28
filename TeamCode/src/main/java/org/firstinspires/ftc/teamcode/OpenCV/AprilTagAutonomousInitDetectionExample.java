/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;


@Autonomous (name = "Auto")
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag IDs of cone
    int onedot = 1;
    int twodot = 2;
    int threedot = 3;

    AprilTagDetection tagOfInterest = null;

    public static double ANGLE = -51; // deg

    public static double CORRECT = 15;
    // VARIABLES FOR THE CLAW
    private Servo cServo = null;

    // VARIABLES FOR THE LIFT
    private DcMotor rlMotor = null;
    private DcMotor llMotor = null;
    private Servo rArm = null;
    private Servo lArm = null;

    //drive
    SampleMecanumDrive drive = null;

    public void setUp() {
// set up the claw
        cServo = hardwareMap.get(Servo.class, "cServo");
        cServo.setDirection(Servo.Direction.REVERSE);
        telemetry.addData("Motors", "right (%.2f)", cServo.getPosition());
        // set up the lifts
        rlMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        llMotor = hardwareMap.get(DcMotor.class, "llMotor");

        //set up the arms
        rArm = hardwareMap.get(Servo.class, "rServo");
        lArm = hardwareMap.get(Servo.class, "lServo");

        // set the direction of the lift motors
        rlMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        llMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // set the motors to run with encoder
        rlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        llMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set target to zero
        rlMotor.setTargetPosition(0);
        llMotor.setTargetPosition(0);

        // stop and reset the encoders
        rlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        llMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the mode to run using position
        rlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        llMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rlMotor.setPower(0.8);
        llMotor.setPower(0.8);

    }

    @Override
    public void runOpMode()
    {
        setUp();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == onedot || tag.id == twodot || tag.id == threedot)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == twodot) { //location 2
            autonomous(2);

        } else if (tagOfInterest.id == onedot) { //location 1
            autonomous(1);

        } else if (tagOfInterest.id == threedot) { //location 3
            autonomous(3);

        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


    public void autonomous(int parkingSpotNumber) {
        //PASTE ENTIRE runOpMode CODE HERE
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // setting up all the motors and servos
        telemetry.addData("Status", "initialized ");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);


        Trajectory forward0 = forward(7); // 7 was old
        Trajectory left = left(11);
        Trajectory forward = forward(19);
        Trajectory right = right(5);
        Trajectory forward2 = forward(25);
        Trajectory back = back(26);
        Trajectory forward3 = forward(14);
        Trajectory forward4 = forward(25);
        Trajectory strafeLeft = left(32);
        Trajectory for16 = drive.trajectoryBuilder(new Pose2d())
                .forward(4)
                .build();
        Trajectory back3 = back(10); // 6 was old
        Trajectory back2 = back(10);
        Trajectory back1 = back(10);

//        Trajectory leftCorrect = left(10);
//        Trajectory strafeHigh = left(52);



        if (isStopRequested()) return;
//        closeClaw();
//        drive.followTrajectory(leftCorrect);
//        drive.turn(-85);
//        drive.followTrajectory(strafeHigh);
//        drive.turn(85);
//        setArms();
//        highJunction();

        closeClaw();
        drive.followTrajectory(forward0);
        setArms();
        sleep(1000);
        drive.turn(Math.toRadians(-37));
       // drive.followTrajectory(for16);
        // set to lower
//        rArm.setPosition(0.6);
//        lArm.setPosition(0.4);
        openClaw();
//        setArms();
        drive.turn(Math.toRadians(37));
        drive.followTrajectory(left);
        drive.turn(Math.toRadians(-70));
        dropArms();
//
//        drive.followTrajectory(strafeLeft);
//        drive.turn(Math.toRadians(175));
//        rArm.setPosition(0.8);
//        lArm.setPosition(0.2);
//        drive.followTrajectory(forward2);
//        closeClaw();
//        sleep(500);
//        outTheBack();
//        drive.followTrajectory(back);
//        drive.turn(Math.toRadians(59));
//        highJunction();
//        drive.followTrajectory(back2);
//        openClaw();
//        sleep(500);
//        dropArms();
//        drive.followTrajectory(forward3);
//        lowJunction();

//        // return back to straight
//        drive.turn(Math.toRadians(50));
//        //go left
//        drive.followTrajectory(left); // done
//        drive.turn(Math.toRadians(CORRECT)); // correct
//        // go forward
//        drive.followTrajectory(forward);
//        // turn 90
//        drive.turn(Math.toRadians(73));
//        // strafe right
//        // drive.followTrajectory(right); possibly not needed
//        // set the arms to the correct position
//        rArm.setPosition(0.8);
//        lArm.setPosition(0.2);
//        // go forward
//        drive.followTrajectory(forward2);
//        // close claw
//        closeClaw();
//        sleep(500);
//        // set arms
//        outTheBack();
//        // go back
//        drive.followTrajectory(back);
//        // turn 50 degrees
//        drive.turn(Math.toRadians(59));
//        // set the junction high
//        highJunction();
//        // move back
//        drive.followTrajectory(back2);
//        // open the claw
//        sleep(500);
//        openClaw();
//        dropArms(); // drop the arms
//        lowJunction(); // drop the junction

        //parking
        if (parkingSpotNumber == 1) { //parking spot 1
//            put parking movement in here
            drive.followTrajectory(strafeLeft);
            drive.followTrajectory(back);
            drive.turn(Math.toRadians(-85));
            drive.followTrajectory(back2);


        }
        else if (parkingSpotNumber == 2) { //parking spot 2
            //put parking movement in here
            drive.followTrajectory(strafeLeft);
            drive.turn(Math.toRadians(-85));
            drive.followTrajectory(back3);

        }
        else if (parkingSpotNumber == 3) { //parking spot 3
            //put parking movement in here
           drive.followTrajectory(strafeLeft);
           drive.followTrajectory(forward);
           drive.turn(Math.toRadians(-85));
           drive.followTrajectory(back1);

            while (!isStopRequested() && opModeIsActive());

        }
    }
    //put extra functions down here
    public void closeClaw() {
        cServo.setPosition(0.2);
    }

    public void openClaw() {
        cServo.setPosition(0.5);
    }

    public void highJunction() {
        rlMotor.setTargetPosition(1143);
        llMotor.setTargetPosition(1143);
    }

    public void lowJunction() {
        rlMotor.setTargetPosition(0);
        llMotor.setTargetPosition(0);
    }

    public void setArms() {
        rArm.setPosition(0.55);
        lArm.setPosition(0.45);
    }

    public void dropArms() {
        rArm.setPosition(0.95);
        lArm.setPosition(0.05);
    }
    public void outTheBack(){
        rArm.setPosition(0.2);
        lArm.setPosition(0.8);
    }

    public Trajectory left(double measurement) {
        return drive.trajectoryBuilder(new Pose2d()).strafeLeft(measurement).build();
    }
    public Trajectory right(double measurement) {
        return drive.trajectoryBuilder(new Pose2d()).strafeRight(measurement).build();
    }

    public Trajectory forward(double measurement) {
        return drive.trajectoryBuilder(new Pose2d()).forward(measurement).build();
    }

    public Trajectory back(double measurement) {
        return drive.trajectoryBuilder(new Pose2d()).back(measurement).build();
    }

    public void arms(double position) {

    }
}
