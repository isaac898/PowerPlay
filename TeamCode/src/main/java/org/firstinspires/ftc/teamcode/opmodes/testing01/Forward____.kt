package org.firstinspires.ftc.teamcode.opmodes.testing01

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.autonomous.BaseAutonomous

@Autonomous
class Forward____ : BaseAutonomous() {
    override fun autonomous() {
        drivetrain.move(y = 20, power = 0.4)
        sleep(1000)
        drivetrain.move(x = -20, power = -0.4) // good

    }
}