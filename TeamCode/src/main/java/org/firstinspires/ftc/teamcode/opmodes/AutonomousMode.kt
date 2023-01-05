package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class AutonomousMode : LinearOpMode() {
    abstract fun initialize()

    open fun begin() {}

    abstract fun autonomous()

    open fun end() {}

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        initialize()

        waitForStart()

        begin()
        autonomous()
        end()
    }
}