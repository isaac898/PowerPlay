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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous
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

    //motor
    DcMotor fl;
    DcMotor bl;
    DcMotor fr;
    DcMotor br;

    //servo
    private Servo claw_servo;
    private Servo right_arm;
    private Servo left_arm;

    public void initialize() {

        //motors
        fl = hardwareMap.dcMotor.get("fl");
        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set up the two arm servos
        right_arm = hardwareMap.get(Servo.class, "rServo");
        left_arm = hardwareMap.get(Servo.class, "lServo");

        // set the arm position to the correct positions
        right_arm.setPosition(0.95);
        left_arm.setPosition(0.05);

        claw_servo = hardwareMap.get(Servo.class, "cServo");
        claw_servo.setDirection(Servo.Direction.REVERSE);
        claw_servo.setPosition(0);

        //odometry

//            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void runOpMode()
    {
        initialize();
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
        if(tagOfInterest == null || tagOfInterest.id == twodot)
        {
            arm();
            sleep(secondsToMilli(1));
            timedTranslate(1, 0.3, 2);
            timedTranslate(1, 0.5, 0);
            timedTranslate(1, 0.5, 0);
            timedTranslate(1, -0.5, 0);
            right_arm.setPosition(0.95);
            left_arm.setPosition(0.05);
        } else if (tagOfInterest.id == onedot) {
            arm();
            sleep(secondsToMilli(1));
            timedTranslate(1, 0.3, 2);
            timedTranslate(1, 0.6, 2);
            sleep(secondsToMilli(1));
            timedTranslate(1, 0.5, 0);
            right_arm.setPosition(0.95);
            left_arm.setPosition(0.05);
        } else if (tagOfInterest.id == threedot) {
            arm();
            sleep(secondsToMilli(1));
            timedTranslate(1, 0.2, 2);
            timedTranslate(1, 0.6, 1);
            sleep(secondsToMilli(1));
            timedTranslate(1, 0.5, 0);
            right_arm.setPosition(0.95);
            left_arm.setPosition(0.05);
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

    public int secondsToMilli(int seconds) {
        int milliseconds = 1000 * seconds;
        return milliseconds;
    }

    public void arm() {
        right_arm.setPosition(0.92); //0.95 lowest
        left_arm.setPosition(0.08); //0.05 lowest
    }

    //seconds can only be int, so change power if needed to shorten distance, 1 = right, 2 = left, 0 = yk

    public void timedTranslate(int seconds, double power, int direction) {
        double flPow = power;
        double blPow = power;
        double frPow = power;
        double brPow = power;
        if (direction == 1) {
            blPow = -power;
            frPow = -power;
        } else if (direction == 2) {
            flPow = -power;
            brPow = -power;
        }

        fl.setPower(flPow);
        fr.setPower(frPow);
        bl.setPower(blPow);
        br.setPower(brPow);

        sleep(secondsToMilli(seconds));
        motorStop();
    }

    public void motorStop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
