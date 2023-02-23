package RoadRunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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

import java.util.Arrays;

@Autonomous
public class AfterMath extends LinearOpMode {
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

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // setting up all the motors and servos
        telemetry.addData("Status", "initialized ");
        setUp();
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);

// The robot maintains the heading it starts at throughout the trajectory
// So, if you start at a 90 degree angle, it will keep that angle the entire path.
//        Trajectory moveLeft = drive.trajectoryBuilder(new Pose2d())
//                .lineToLinearHeading(new Pose2d(0,-100, Math.toRadians(-10)))
//                .build();

        Trajectory moveRight = drive.trajectoryBuilder(new Pose2d())
                .lineToConstantHeading(new Vector2d(0, -52))
                .build();

        Trajectory moveBack = drive.trajectoryBuilder(new Pose2d()).back(10).build();

        Trajectory backHoe = drive.trajectoryBuilder(new Pose2d()).back(10).build();


        Trajectory SENDIT = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(8)))
                .build();
        Trajectory SENDITT = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(12,0,Math.toRadians(-25)))
                .build();
        Trajectory jimmy  = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-12,0,Math.toRadians(25)))
                .build();
        Trajectory jim = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-10,0,Math.toRadians(-8)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // segment one
        // pick up the cone
        closeClaw();
        sleep(150);
        liftaLittle();
        // drive to the juntion
        drive.followTrajectory(moveRight);
        drive.turn(Math.toRadians(8));
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
        drive.turn(Math.toRadians(20));
        drive.followTrajectory(backHoe);
        sleep(150);
        openClaw();
        totalReset();


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


    public void setUp(){
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

}
