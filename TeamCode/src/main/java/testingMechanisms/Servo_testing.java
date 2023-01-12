package testingMechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo_testing")

public class Servo_testing extends OpMode {
    private Servo clawServo;
    boolean flag = true;
    double position = 0;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        clawServo = hardwareMap.get(Servo.class, "cServo");
        clawServo.setDirection(Servo.Direction.REVERSE);
        clawServo.setPosition(0);
        telemetryUpdate();

        telemetry.update();
    }

    @Override
    public void loop() {
        if(gamepad1.x){
            if(flag){
                position +=0.1;
                clawServo.setPosition(position);
                telemetryUpdate();
                flag = false;
            }
        } else if (gamepad1.b){
            if(flag){
                position -= 0.1;
                clawServo.setPosition(position);
                telemetryUpdate();
                flag = false;
            }
        } else {
            flag = true;
            telemetryUpdate();
        }

    }

    public void telemetryUpdate() {
        telemetry.addData("Motors", "claw position (%.2f)", clawServo.getPosition());
        telemetry.update();
    }
}
