package org.firstinspires.ftc.teamcode.opmodes.testing


import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.autonomous.BaseAutonomous

@Autonomous
class ChangeInAll : BaseAutonomous() {
    override fun autonomous() {
        drivetrain.odometry()
        drivetrain.move(20, 40, 90,0.6 )
        drivetrain.telemetry()

    }
}