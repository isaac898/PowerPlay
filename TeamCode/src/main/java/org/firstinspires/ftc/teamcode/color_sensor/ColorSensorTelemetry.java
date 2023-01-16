package org.firstinspires.ftc.teamcode.color_sensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ColorSensorTelemetry extends OpMode {

    ColorSensor clawSensorColor;
    DistanceSensor clawSensorDistance;

    double red, blue, green, distance;

    @Override
    public void init() {
        clawSensorColor = hardwareMap.get(ColorSensor.class, "Claw Sensor");
        clawSensorDistance = hardwareMap.get(DistanceSensor.class, "Claw Sensor");

    }

    @Override
    public void loop() {
        red = clawSensorColor.red();
        blue = clawSensorColor.blue();
        green = clawSensorColor.green();
        distance = clawSensorDistance.getDistance(DistanceUnit.MM);

        telemetry.addData("R", red);
        telemetry.addData("G", green);
        telemetry.addData("B", blue);
        telemetry.addData("Distance", distance);

        telemetry.update();
    }
}
