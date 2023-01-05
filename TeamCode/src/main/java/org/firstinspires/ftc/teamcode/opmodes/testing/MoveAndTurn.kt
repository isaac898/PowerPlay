package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.autonomous.BaseAutonomous

@Autonomous
class MoveAndTurn : BaseAutonomous() {
    override fun autonomous() {
        drivetrain.move(36, 36, 180)
    }
}