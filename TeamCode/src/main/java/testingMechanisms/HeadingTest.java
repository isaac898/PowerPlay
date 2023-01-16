package testingMechanisms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class HeadingTest extends OpMode {
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor br;
    private DcMotor bl;
    private BNO055IMU imu;

    @Override
    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        float firtstAngle = imu.getAngularOrientation().firstAngle;

        List<Float> allAngles;
        allAngles = new ArrayList<Float>();

        allAngles.add(imu.getAngularOrientation().firstAngle);
        allAngles.add(imu.getAngularOrientation().secondAngle);
        allAngles.add(imu.getAngularOrientation().thirdAngle);

        /*
        the IMU built into the expansion hub is used to track the rotation of the robot.
        The IMU get output three angles, so we take the first, second, and third angle and
        store them in an arrayList
        */

        double lastAngle = 0.0;
        double heading = 0.0;

        double changeInHeading;





    }

    @Override
    public void loop() {

    }

}
