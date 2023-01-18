package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends OpMode {

    private Servo rArm = null;
    private Servo lArm = null;
    double rPos = 0;
    double lPos = 0;

    @Override
    public void init() {
        rArm = hardwareMap.get(Servo.class, "rServo");
        lArm = hardwareMap.get(Servo.class, "lServo");

        rArm.setDirection(Servo.Direction.FORWARD);
        lArm.setDirection(Servo.Direction.REVERSE);

        rArm.setPosition(0);
        lArm.setPosition(0);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            rPos += 0.001;
        }
        else if (gamepad1.a) {
            rPos -= 0.001;
        }
        else if (gamepad1.left_bumper) {
            lPos += 0.001;
        }
        else if (gamepad1.b) {
            lPos -= 0.001;
        }

        rArm.setPosition(rPos);
        lArm.setPosition(lPos);

        updateTelemetry();
    }

    public void updateTelemetry() {
        telemetry.addData("rPos: ", rArm.getPosition());
        telemetry.addData("lPos: ", lArm.getPosition());
        telemetry.update();
    }
}
