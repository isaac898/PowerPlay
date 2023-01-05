package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.autonomous.BaseAutonomous

@Autonomous
class ChangeInY : BaseAutonomous() {
    override fun autonomous() {
        drivetrain.odometry()
        drivetrain.move(0, 20, 0,0.7 )
        drivetrain.telemetry()

    }
}