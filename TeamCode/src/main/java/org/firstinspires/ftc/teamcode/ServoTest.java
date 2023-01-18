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

    private DcMotor rlMotor = null;
    private DcMotor llMotor = null;

    double rPos = 0;
    double lPos = 0;

    double pow = 0;

    @Override
    public void init() {
        rArm = hardwareMap.get(Servo.class, "rServo");
        lArm = hardwareMap.get(Servo.class, "lServo");

        rlMotor = hardwareMap.get(DcMotor.class, "rlMotor");
        llMotor = hardwareMap.get(DcMotor.class, "llMotor");

        rlMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

        rArm.setDirection(Servo.Direction.FORWARD);
        lArm.setDirection(Servo.Direction.REVERSE);

        rArm.setPosition(0);
        lArm.setPosition(0);
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

        pow = gamepad1.right_stick_y * 0.5;

        rArm.setPosition(rPos);
        lArm.setPosition(lPos);

        llMotor.setPower(pow);
        rlMotor.setPower(pow);

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
