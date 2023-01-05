package org.firstinspires.ftc.teamcode.mechanisms


import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.*
import kotlin.math.abs

class Carousel : Mechanism {
    private companion object {
        // chosen

        const val SLOW_POWER = 0.52
        const val FAST_POWER = 0.9
        const val SLOW_CAROUSEL_ROTATIONS = 0.7
        const val FAST_CAROUSEL_ROTATIONS = 0.4

        // measured

        const val COUNTS_PER_WHEEL_ROTATION = 537.6

        const val WHEEL_DIAMETER_INCHES = 4.0
        const val CAROUSEL_DIAMETER_INCHES = 15.0

        // derived

        const val COUNTS_PER_CAROUSEL_ROTATION =
            COUNTS_PER_WHEEL_ROTATION / WHEEL_DIAMETER_INCHES * CAROUSEL_DIAMETER_INCHES

        const val SLOW_COUNTS = SLOW_CAROUSEL_ROTATIONS * COUNTS_PER_CAROUSEL_ROTATION
        const val FAST_COUNTS =
            (SLOW_CAROUSEL_ROTATIONS + FAST_CAROUSEL_ROTATIONS) * COUNTS_PER_CAROUSEL_ROTATION
    }

    private lateinit var carousel: DcMotorEx

    override fun initialize() {
        carousel = hardwareMap.get(DcMotorEx::class.java, ::carousel.name)

        carousel.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        carousel.targetPosition = 0

        carousel.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        carousel.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun update() {
        when {
            gamepad1.dpad_left ->
                deliver()
            gamepad1.dpad_right ->
                slow()
            else -> {
                off()

                flag = true
            }
        }
    }

    private var startPosition = 0
    private var flag = true
    fun deliver(autonomous: Boolean = false): Boolean {
        if (flag) {
            startPosition = carousel.currentPosition

            flag = false
        }

        val position = abs(carousel.currentPosition - startPosition)
        when {
            !autonomous && position < SLOW_COUNTS -> {
                slow()
            }
            position < FAST_COUNTS + if (autonomous) SLOW_COUNTS * 2.0 else 0.0 -> {
                if (autonomous) {
                    auto()
                } else {
                    fast()
                }
            }
            else -> {
                off()
                return true
            }
        }
        return false
    }

    private fun slow() {
        carousel.power = alliance.value * SLOW_POWER
    }

    private fun auto() {
        carousel.power = alliance.value * 0.25
    }

    private fun fast() {
        carousel.power = alliance.value * FAST_POWER
    }

    private fun off() {
        carousel.power = 0.0
    }

    override fun telemetry() {
        telemetry.addData("carousel power", carousel.power)
        telemetry.addData("carousel position", carousel.currentPosition)
        telemetry.addData("slow counts", SLOW_COUNTS)
    }
}