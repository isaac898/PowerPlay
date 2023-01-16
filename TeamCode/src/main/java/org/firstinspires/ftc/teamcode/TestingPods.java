package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestingPods extends OpMode {
    private DcMotor mOdometryPod;
    private DcMotor rOdometryPod;
    private DcMotor lOdometryPod;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        mOdometryPod = hardwareMap.get(DcMotor.class, "middle");
        rOdometryPod = hardwareMap.get(DcMotor.class, "right");
        lOdometryPod = hardwareMap.get(DcMotor.class, "left");

        telemetry.update();



    }

    @Override
    public void loop() {

        telemetry.addData("Middle Pod", mOdometryPod.getCurrentPosition());
        telemetry.addData("Right Pod", rOdometryPod.getCurrentPosition());
        telemetry.addData("Left Pod", lOdometryPod.getCurrentPosition());
// update not needed because it is in teleOp mode

    }
}
