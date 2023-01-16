package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class OneMotor extends OpMode {
    private DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        motor.setPower(0.8);

    }
}
