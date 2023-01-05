package org.firstinspires.ftc.teamcode.opmodes.testing


import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.autonomous.BaseAutonomous

@Autonomous
class BasicMovement : BaseAutonomous() {
    override fun autonomous() {
        telemetry.addData("Top one had started running: ", 0)

        drivetrain.move(power = 0.8)

        drivetrain.move(x = -2 )

        drivetrain.move(y = -2)

        drivetrain.move(heading = 180)


        telemetry.addData("bottom one had finished running: ", 0)

    }
}