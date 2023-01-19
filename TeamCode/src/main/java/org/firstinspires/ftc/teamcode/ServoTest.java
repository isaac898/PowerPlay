package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends OpMode {

    private Servo rArm = null;
    private Servo lArm = null;

    private Servo cServo = null;


    private DcMotor rlMotor = null;
    private DcMotor llMotor = null;

    double rPos = 0;
    double lPos = 0;

    @Override
    public void init() {
        cServo = hardwareMap.get(Servo.class, "cServo");
        cServo.setDirection(Servo.Direction.REVERSE);

        cServo.setPosition(0.2);

        rArm = hardwareMap.get(Servo.class, "rServo");
        lArm = hardwareMap.get(Servo.class, "lServo");

        rlMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        llMotor = hardwareMap.get(DcMotor.class, "llMotor");

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

        rArm.setDirection(Servo.Direction.REVERSE);
        lArm.setDirection(Servo.Direction.FORWARD);

        rArm.setPosition(0);
        lArm.setPosition(0);

        rlMotor.setPower(0.8);
        llMotor.setPower(0.8);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            lPos += 0.001;
            rPos += 0.001;

        }
        else if (gamepad1.dpad_down) {
            lPos -= 0.001;
            rPos -= 0.001;
        }

        else if (gamepad1.y) {
            rlMotor.setTargetPosition(1000);
            llMotor.setTargetPosition(1000);
        }

        else if (gamepad1.a) {
            rlMotor.setTargetPosition(0);
            llMotor.setTargetPosition(0);
        }

        rArm.setPosition(rPos);
        lArm.setPosition(lPos);

        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addData("rPos: ", rArm.getPosition());
        telemetry.addData("lPos: ", lArm.getPosition());
        telemetry.addData("rPos", rlMotor.getCurrentPosition());
        telemetry.addData("lPos", llMotor.getCurrentPosition());
        telemetry.update();
    }
}
