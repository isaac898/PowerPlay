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
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;


@Autonomous (name = "Auto14")
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
//        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);


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

        Trajectory moveRight = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(0, -52))
                .build();

        Trajectory moveBack = drive.trajectoryBuilder(new Pose2d()).back(9).build();

        Trajectory backHoe = drive.trajectoryBuilder(new Pose2d()).back(5).build();


        Trajectory SENDIT = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(8)))
                .build();
        Trajectory SENDITT = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(12,0,Math.toRadians(-22)))
                .build();
        Trajectory jimmy  = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-12,0,Math.toRadians(22)))
                .build();
        Trajectory jim = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-10,0,Math.toRadians(-8)))
                .build();

        Trajectory bbs = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(10,0,Math.toRadians(-37)))
                .build();

        Trajectory fish = drive.trajectoryBuilder(new Pose2d())
                .forward(13)
                .build();

        Trajectory swamp = drive.trajectoryBuilder(new Pose2d())
                .forward(15)
                .build();
        Trajectory biscuit = drive.trajectoryBuilder(new Pose2d())
                .back(15)
                .build();




        waitForStart();

        if (isStopRequested()) return;


        //parking
        if (parkingSpotNumber == 1) { //parking spot 1
            // segment one
            // pick up the cone
            closeClaw();
            sleep(150);
            liftaLittle();
            // drive to the juntion
            drive.followTrajectory(moveRight);
            drive.turn(Math.toRadians(12));
            highJunction();
            sleep(250);
            outTheBack();
            sleep(250);
            // drop the cone
            drive.followTrajectory(moveBack);
            sleep(250);
            openClaw();
            sleep(150);
            // operation status: in progress

            totalReset();
            sleep(250);

            drive.followTrajectory(fish);
            drive.turn(Math.toRadians(-12));
            drive.followTrajectory(swamp);


        }
        else if (parkingSpotNumber == 2) { //parking spot 2
            // segment one
            // pick up the cone
            closeClaw();
            sleep(150);
            liftaLittle();
            // drive to the juntion
            drive.followTrajectory(moveRight);
            drive.turn(Math.toRadians(12));
            highJunction();
            sleep(250);
            outTheBack();
            sleep(250);
            // drop the cone
            drive.followTrajectory(moveBack);
            sleep(250);
            openClaw();
            sleep(150);
            // operation status: in progress

            totalReset();
            sleep(250);

            // get that damn cone and score it
            drive.followTrajectory(SENDIT);
            firstCone();
            drive.followTrajectory(SENDITT);
            closeClaw();
            sleep(150);
            highJunction();
            drive.followTrajectory(jimmy);
            outTheBack();
            drive.followTrajectory(jim);
            drive.turn(Math.toRadians(24));
            drive.followTrajectory(backHoe);
            sleep(150);
            openClaw();
            sleep(250);
            totalReset();

            drive.followTrajectory(bbs);

        }
        else if (parkingSpotNumber == 3) { //parking spot 3
            // segment one
            // pick up the cone
            closeClaw();
            sleep(150);
            liftaLittle();
            // drive to the juntion
            drive.followTrajectory(moveRight);
            drive.turn(Math.toRadians(12));
            highJunction();
            sleep(250);
            outTheBack();
            sleep(250);
            // drop the cone
            drive.followTrajectory(moveBack);
            sleep(250);
            openClaw();
            sleep(150);
            // operation status: in progress

            totalReset();
            sleep(250);

            drive.followTrajectory(fish);
            drive.turn(Math.toRadians(-12));
            drive.followTrajectory(biscuit);



        }
        while (!isStopRequested() && opModeIsActive());

    }
    public void closeClaw() {
        cServo.setPosition(0.2);
    }

    public void openClaw() {
        cServo.setPosition(0.43);
    }

    public void reachForIt() {
        rlMotor.setTargetPosition(1595);
        llMotor.setTargetPosition(1595);
    }

    public void reset(){
        closeClaw();
        sleep(250);
        firstCone();
//        midPoint();
        sleep(250);
        low();
        sleep(300);
        openClaw();

    }

    public void totalReset(){
        closeClaw();
        sleep(250);
        resetArms();
        sleep(250);
        low();

    }

    public void firstCone(){
        rArm.setPosition(0.8);
        lArm.setPosition(0.2);
        openClaw();
    }
    public void midPoint(){
        rArm.setPosition(0.65);
        lArm.setPosition(0.35);
    }

    public void highJunction() {
        rlMotor.setTargetPosition(1647);
        llMotor.setTargetPosition(1647);
    }
    public void liftaLittle(){
        rlMotor.setTargetPosition(300);
        llMotor.setTargetPosition(300);
    }

    public void low() {
        rlMotor.setTargetPosition(0);
        llMotor.setTargetPosition(0);
    }

    public void resetArms() {
        rArm.setPosition(1);
        lArm.setPosition(0);
    }

    public void dropArms() {
        rArm.setPosition(0.95);
        lArm.setPosition(0.05);
    }
    public void outTheBack(){
        rArm.setPosition(0.3);
        lArm.setPosition(0.7);
    }


}
