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

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // setting up all the motors and servos
        telemetry.addData("Status", "initialized ");
        setUp();
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);

        //needed voltage = 14.21

        // segment one variables
        Trajectory left = left(89);
        Trajectory back1 = back(9);
        // segment one variables all work ( if it doesn't work then its the angle that you start at or the power

        // segment two variables
        Trajectory forward1 = forward(6);
        Trajectory right = right(20); // might change to 20
        Trajectory forward2 = forward(22);
        Trajectory back2 = back(23);
        Trajectory left1 = left(20.5);
        Trajectory back3 = back(10);
        // variables all work for a power -- we have been mainly using 14.38 to 14.39
        // but the recent trial showed that even 14.3 worked

        // last forward
        Trajectory finalForward = forward(8);


        waitForStart();

        if (isStopRequested()) return;
// segment one :: score on the high junction
        closeClaw();
        sleep(250);
        liftaLittle();
        drive.followTrajectory(left);
        highJunction();
        sleep(250);
        outTheBack();
        outTheBack();
        sleep(250);
        drive.followTrajectory(back1);
        sleep(250);
        openClaw();
        // segment one works

        // segment two :: pick up the first cone of the stakc and score it
        drive.followTrajectory(forward1);
        drive.followTrajectory(right); // might need to be adjusted
        drive.turn(Math.toRadians(21));
        sleep(150);
        reset();
        drive.followTrajectory(forward2);
        sleep(150);
        closeClaw();
//        reachForIt();
//        sleep(250);
//        drive.followTrajectory(back2);
//        outTheBack();
//        sleep(250);
//        drive.followTrajectory(left1);
//        drive.followTrajectory(back3);
//        sleep(250);
//        openClaw();
        // segment two works okay, some adjustments might need to be made

//        // segment three
//        drive.followTrajectory(forward1);
//        drive.followTrajectory(right); // might need to be adjusted
//        reset();
//        drive.followTrajectory(forward2);
//        sleep(150);
//        closeClaw();
//        reachForIt();
//        sleep(250);
//        drive.followTrajectory(back2);
//        outTheBack();
//        sleep(250);
//        drive.followTrajectory(left1);
//        drive.followTrajectory(back3);
//        sleep(250);
//        openClaw();
//
//        // end
        drive.followTrajectory(finalForward);
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
