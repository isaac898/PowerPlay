package testingMechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "arm_testing")

public class arm_testing extends OpMode {
    private Servo leftServo;
    private Servo rightServo;
    boolean flag = true;
    double leftPosition = 0;
    double rightPosition = 1;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        leftServo.setPosition(0.1);
        rightServo.setPosition(0.9);

        telemetryUpdate();

    }

    @Override
    public void loop() {
        if(gamepad1.x){
            if(flag){
                leftPosition += 0.1;
                rightPosition -= 0.1;

                leftServo.setPosition(leftPosition);
                rightServo.setPosition(rightPosition);

                telemetryUpdate();

                flag = false;
            }
        } else if (gamepad1.b){
            if(flag){
                leftPosition -= 0.1;
                rightPosition += 0.1;

                leftServo.setPosition(leftPosition);
                rightServo.setPosition(rightPosition);

                telemetryUpdate();

                flag = false;
            }
        } else {
            flag = true;
            telemetryUpdate();
        }

    }
    public void telemetryUpdate() {
        telemetry.addData("Motors", "right_servo (%.2f)", rightServo.getPosition());
        telemetry.addData("Morots", "left_servo(%.2f)", leftServo.getPosition());
        telemetry.update();

    }
}
