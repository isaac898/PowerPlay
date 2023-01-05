package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.autonomous.BaseAutonomous

@Autonomous
class ChangeInX : BaseAutonomous() {
    override fun autonomous() {
        drivetrain.odometry()
        drivetrain.move(20, 0, 0,0.7 )
        drivetrain.telemetry()

    }
}