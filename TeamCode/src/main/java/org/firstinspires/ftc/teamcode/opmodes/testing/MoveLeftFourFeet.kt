package org.firstinspires.ftc.teamcode.opmodes.testing


import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.autonomous.BaseAutonomous as NeutralAutonomous

@Autonomous
class MoveLeftFourFeet : NeutralAutonomous() {
    override fun autonomous() {
        drivetrain.move(x = -2)
    }
}