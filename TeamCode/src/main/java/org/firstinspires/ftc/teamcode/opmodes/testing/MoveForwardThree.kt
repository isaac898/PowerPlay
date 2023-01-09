package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.autonomous.BaseAutonomous

@Autonomous
class MoveForwardThree : BaseAutonomous() {
    override fun autonomous() {
        drivetrain.move(y = 48, heading = 2.1) // goes to the third box, y = 48
        //sleep(1000)
        //drivetrain.move()
    }
}