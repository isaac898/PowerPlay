/*
* Copyright (c) 2021 OpenFTC Team
* deal
*
* The above copyright notice and this permission notice shall be included in
all
FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE
* SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.OpenCV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous
public class Parking extends LinearOpMode {
    // VARIABLES FOR THE CLAW
    private Servo cServo = null;

    // VARIABLES FOR THE LIFT
    private DcMotor rlMotor = null;
    private DcMotor llMotor = null;
    private Servo rArm = null;
    private Servo lArm = null;

    //drive
    SampleMecanumDrive drive = null;

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
//    //servo
//    private Servo claw_servo;
//    private Servo right_arm;
//    private Servo left_arm;

    public void initialize() {
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

//odometry
// fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void runOpMode() {
        initialize();


        drive = new SampleMecanumDrive(hardwareMap);

        // segment one variables
        Trajectory left = left(88);
        Trajectory back1 = back(6);
        // segment one variables all work ( if it doesn't work then its the angle that you start at or the power

        // segment two variables
        Trajectory forward1 = forward(6);
        Trajectory right = right(19); // might change to 20
        Trajectory forward2 = forward(22);
        Trajectory back2 = back(23);
        Trajectory left1 = left(20.5);
        Trajectory back3 = back(8.3);
        // variables all work for a power that is at 14.1 ~ starting value

        // last forward
        Trajectory finalForward = forward(8);


        waitForStart();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == onedot || tag.id == twodot || tag.id == threedot) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
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
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        closeClaw();
        sleep(250);
        liftaLittle();
        drive.followTrajectory(left);
        highJunction();
        sleep(250);
        outTheBack();
        sleep(250);
        drive.followTrajectory(back1);
        sleep(250);
        openClaw();
        // segment one works

        // segment two :: pick up the first cone of the stakc and score it
        drive.followTrajectory(forward1);
        drive.followTrajectory(right); // might need to be adjusted
        reset();
        drive.followTrajectory(forward2);
        sleep(150);
        closeClaw();
        reachForIt();
        sleep(250);
        drive.followTrajectory(back2);
        outTheBack();
        sleep(250);
        drive.followTrajectory(left1);
        drive.followTrajectory(back3);
        sleep(250);
        openClaw();
        // segment two works okay, some adjustments might need to be made


//        // end
        drive.followTrajectory(finalForward);
        totalReset();

        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == twodot) { //location 2

        } else if (tagOfInterest.id == onedot) {

        } else if (tagOfInterest.id == threedot) {

        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public int secondsToMilli(int seconds) {
        int milliseconds = 1000 * seconds;
        return milliseconds;
    }




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
        openClaw();
    }

    public void firstCone(){
        rArm.setPosition(0.8);
        lArm.setPosition(0.2);
    }
    public void midPoint(){
        rArm.setPosition(0.65);
        lArm.setPosition(0.35);
    }

    public void highJunction() {
        rlMotor.setTargetPosition(1547);
        llMotor.setTargetPosition(1547);
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

}

