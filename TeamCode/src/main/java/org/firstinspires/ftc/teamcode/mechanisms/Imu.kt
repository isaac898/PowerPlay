package org.firstinspires.ftc.teamcode.mechanisms


import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.teamcode.hardwareMap
import kotlin.math.abs
import kotlin.math.sign

class Imu : Mechanism {
    private lateinit var imu: BNO055IMU

    override fun initialize() {
        val parameters = BNO055IMU.Parameters().apply { angleUnit = BNO055IMU.AngleUnit.DEGREES }

        imu = hardwareMap.get(BNO055IMU::class.java, ::imu.name)

        imu.initialize(parameters)
    }

    val firstAngle get() = imu.angularOrientation.firstAngle.toDouble()

    val allAngles
        get() = Triple(
            imu.angularOrientation.firstAngle.toDouble(),
            imu.angularOrientation.secondAngle.toDouble(),
            imu.angularOrientation.thirdAngle.toDouble()
        )

    private var lastAngle = 0.0
    var heading = 0.0
        get() {
            var changeHeading = firstAngle - lastAngle

            if (abs(changeHeading) > 180.0) changeHeading -= sign(changeHeading) * 360.0

            field += changeHeading

            lastAngle = firstAngle
            return field
        }
}