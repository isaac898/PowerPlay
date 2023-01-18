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
public class LowThenCycle extends LinearOpMode {
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

        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);
    }

    //map out path
    public void movementCodes() {


    }

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // setting up all the motors and servos
        telemetry.addData("Status", "initialized ");

        initialize();

        Trajectory for16 = drive.trajectoryBuilder(new Pose2d())
                .forward(7)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        closeClaw();
        drive.followTrajectory(for16);
        setArms();
        sleep(1000);
        drive.turn(Math.toRadians(-50));
        openClaw();


        // return back to straight
        drive.turn(Math.toRadians(50));
        //go left
        drive.followTrajectory(left(17));
        drive.turn(Math.toRadians(CORRECT)); // correct
        dropArms();
        // go forward
        drive.followTrajectory(forward(48));
        // turn 90
        drive.turn(Math.toRadians(90));
        // strafe right
        drive.followTrajectory(right(5));
        // set the arms to the correct position
        rArm.setPosition(0.85);
        lArm.setPosition(0.15);
        // go forward
        drive.followTrajectory(forward(26));
        // close claw
        closeClaw();
        // set arms
        rArm.setPosition(0.45);
        lArm.setPosition(0.55);
        // go back
        drive.followTrajectory(back(26));
        // turn 50 degrees
        drive.turn(Math.toRadians(70));



        while (!isStopRequested() && opModeIsActive());


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
        rArm.setPosition(0.7);
        lArm.setPosition(0.3);
    }

    public void dropArms() {
        rArm.setPosition(0.95);
        lArm.setPosition(0.05);
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

    public void translate(Trajectory movementCode) {
        drive.followTrajectory(movementCode);
    }
    public void arms(double position) {

    }

}
