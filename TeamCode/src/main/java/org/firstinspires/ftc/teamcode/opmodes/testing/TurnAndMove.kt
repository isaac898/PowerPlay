package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.autonomous.BaseAutonomous


@Autonomous
class TurnAndMove : BaseAutonomous(){
    override  fun autonomous() {
        drivetrain.move(y = 12, heading = 90)
    }
}