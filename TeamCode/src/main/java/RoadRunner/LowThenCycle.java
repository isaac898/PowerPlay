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

        Trajectory for16 = drive.trajectoryBuilder(new Pose2d())
                .forward(7)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        closeClaw();
        setArms();
        drive.followTrajectory(for16);
        drive.turn(Math.toRadians(-50));
        openClaw();


        // return back to straight
        drive.turn(Math.toRadians(50));
        //go left
        Trajectory left = drive.trajectoryBuilder(new Pose2d()).strafeLeft(17).build();
        drive.followTrajectory(left);
        drive.turn(Math.toRadians(CORRECT)); // correct
        dropArms();
        // go forward
        Trajectory forward = drive.trajectoryBuilder(new Pose2d()).forward(48).build();
        drive.followTrajectory(forward);
        // turn 90
        drive.turn(Math.toRadians(90));
        // strafe right
        Trajectory right = drive.trajectoryBuilder(new Pose2d()).strafeRight(5).build();
        drive.followTrajectory(right);
        // set the arms to the correct position
        rArm.setPosition(0.85);
        lArm.setPosition(0.15);
        // go forward
        Trajectory forward2 = drive.trajectoryBuilder(new Pose2d()).forward(20).build();
        drive.followTrajectory(forward2);
        // close claw
        closeClaw();
        // set arms
        rArm.setPosition(0.45);
        lArm.setPosition(0.55);
        // go back
        Trajectory back = drive.trajectoryBuilder(new Pose2d()).back(20).build();
        drive.followTrajectory(back);



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

}
