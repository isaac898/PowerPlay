package RoadRunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class Trajectories extends LinearOpMode {
    public static double ANGLE = -51; // deg

    public static double CORRECT = 15;
    // VARIABLES FOR THE CLAW
    private Servo cServo;

    // VARIABLES FOR THE LIFT
    private DcMotor rlMotor;
    private DcMotor llMotor;
    private Servo rArm;
    private Servo lArm;


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // setting up all the motors and servos
        telemetry.addData("Status", "initialized ");

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

        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory for55 = drive.trajectoryBuilder(new Pose2d())
                .forward(50)
                .build();
        Trajectory for16 = drive.trajectoryBuilder(new Pose2d())
                .forward(16)
                .build();

        Trajectory back = drive.trajectoryBuilder(new Pose2d())
                .back(10)
                .build();
        Trajectory left = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(17)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        // scoring for the first junction
        closeClaw(); // closes the claw
        drive.followTrajectory(left);
        drive.turn(Math.toRadians(CORRECT));
        setArms();
        drive.followTrajectory(for55); // goes forward 12 inches
        highJunction();
        drive.turn(Math.toRadians(ANGLE)); // changes 57 degrees right
        drive.followTrajectory(for16);
        openClaw();
        drive.followTrajectory(back);
        drive.turn(Math.toRadians(60)); // goes 60 degrees left
        lowJunction();
        dropArms();
        // go back to being straight
        drive.turn(Math.toRadians(90));  // go enough for now
        Trajectory for27 = drive.trajectoryBuilder(new Pose2d()).forward(27).build();
        drive.followTrajectory(for27);

        // arm position for the first cone
        rArm.setPosition(0.85);
        lArm.setPosition(0.15);
        Trajectory for10 = drive.trajectoryBuilder(new Pose2d()).forward(10).build();
        drive.followTrajectory(for10);
        closeClaw();
        setArms();
        Trajectory bfor40 = drive.trajectoryBuilder(new Pose2d()).back(34).build();
        drive.followTrajectory(bfor40);


//        // arm position for the second cone
//        rArm.setPosition(0.85);
//        lArm.setPosition(0.15);
//
//        // arm position for the third cone
//        rArm.setPosition(0.85);
//        lArm.setPosition(0.15);
//
//        // arm position for the fourth cone
//        rArm.setPosition(0.9);
//        lArm.setPosition(0.1);
//
//        //arm position for the fifth cone
//        rArm.setPosition(1);
//        lArm.setPosition(0);


        while (!isStopRequested() && opModeIsActive()) ;

        /*
        first cone: right needs to be 0.85
        first cone: left needs to be 0.15

        second cone: right needs to be 0.85
        second cone: left needs to be 0.15
        needs to go a little bit forward

        third cone: right needs to be 0.85
        third cone: left needs to be 0.15
        needs to go a little bit forward from the second cone

        fourth cone: right needs to be 0.9
        fourth cone: left needs to be 0.1
        robot needs to go a little bit forward

        fifth cone: right needs to be 1
        fifth cone: left needs to be 0
        robot needs to come a little bit forward
        */

        //        Pose2d poseEstimate = drive.getPoseEstimate();
//        telemetry.addData("finalX", poseEstimate.getX());
//        telemetry.addData("finalY", poseEstimate.getY());
//        telemetry.addData("finalHeading", poseEstimate.getHeading());
//        telemetry.addData("Servo Position", "Claw position: (%5.2f)", cServo.getPosition());
//        telemetry.addData("Servo", "right arm:  (%.2f)", rArm.getPosition());
//        telemetry.addData("Servo", "left arm: (%.2f)", lArm.getPosition());
//        telemetry.addData("Encoder count Right: ", +rlMotor.getCurrentPosition());
//        telemetry.addData("Encoder Counts Left: ", +llMotor.getCurrentPosition());
//        telemetry.update();

    }

    public void closeClaw() {
        cServo.setPosition(0.2);
    }

    public void openClaw() {
        cServo.setPosition(0.5);
    }

    public void highJunction() {
        rlMotor.setTargetPosition(1000);
        llMotor.setTargetPosition(1000);
    }

    public void lowJunction() {
        rlMotor.setTargetPosition(0);
        llMotor.setTargetPosition(0);
    }

    public void setArms() {
        rArm.setPosition(0.6);
        lArm.setPosition(0.4);
    }

    public void dropArms() {
        rArm.setPosition(0.95);
        lArm.setPosition(0.05);
    }

}
