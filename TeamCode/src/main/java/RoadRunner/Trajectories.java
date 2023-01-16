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
    public static double ANGLE = -35; // deg
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory for52 = drive.trajectoryBuilder(new Pose2d())
                .forward(52)
                .build();
        Trajectory for40 = drive.trajectoryBuilder(new Pose2d())
                .forward(40)
                .build();

        // setting up all the motors and servos
        telemetry.addData("Status", "initialized ");

        // set up the claw
        cServo = hardwareMap.get(Servo.class, "cServo");
        cServo.setDirection(Servo.Direction.REVERSE);
        telemetry.addData("Motors","right (%.2f)", cServo.getPosition());
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


        waitForStart();

        if (isStopRequested()) return;

        // beginning of the code
        closeClaw(); // closes the claw
        drive.followTrajectory(for52); // goes forward 12 inches
        sleep(5000); // wait 5 seconds
        drive.turn(Math.toRadians(ANGLE)); // changes 90 degrees
        setArms(); // brings arms up
        highJunction(); // brings the lift up
        drive.followTrajectory(for40); // goes forward 40 inches
        dropArms(); // brings arms down
        lowJunction(); // brings the lift down
        openClaw(); // opens the claw


        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.addData("Servo Position", "Claw position: (%5.2f)" , cServo.getPosition());
        telemetry.addData("Servo", "right arm:  (%.2f)", rArm.getPosition());
        telemetry.addData("Servo", "left arm: (%.2f)", lArm.getPosition());
        telemetry.addData("Encoder count Right: ", + rlMotor.getCurrentPosition());
        telemetry.addData("Encoder Counts Left: ", + llMotor.getCurrentPosition());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;


    }

    public void closeClaw(){
        cServo.setPosition(0.2);
    }
    public void openClaw(){
        cServo.setPosition(0.5);
    }

    public void highJunction(){
        rlMotor.setTargetPosition(1000);
        llMotor.setTargetPosition(1000);
    }
    public void lowJunction(){
        rlMotor.setTargetPosition(0);
        llMotor.setTargetPosition(0);
    }
    public void setArms(){
        rArm.setPosition(0.6);
        lArm.setPosition(0.4);
    }
    public void dropArms(){
        rArm.setPosition(0.95);
        lArm.setPosition(0.05);
    }

}
