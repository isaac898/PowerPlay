package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.opmodes.autonomous.BaseAutonomous

@Autonomous
class MoveForwardThree : BaseAutonomous() {
    private lateinit var fl: DcMotorEx
    private lateinit var fr: DcMotorEx
    private lateinit var bl: DcMotorEx
    private lateinit var br: DcMotorEx
    private lateinit var motors: List<DcMotorEx>
    private val brake: Boolean = true


    override fun autonomous() {
        drivetrain.move(y = 48, heading = 2.1) // goes to the third box, y = 48

        sleep(1000)

        fl = org.firstinspires.ftc.teamcode.hardwareMap.get(DcMotorEx::class.java, ::fl.name)
        fr = org.firstinspires.ftc.teamcode.hardwareMap.get(DcMotorEx::class.java, ::fr.name)
        bl = org.firstinspires.ftc.teamcode.hardwareMap.get(DcMotorEx::class.java, ::bl.name)
        br = org.firstinspires.ftc.teamcode.hardwareMap.get(DcMotorEx::class.java, ::br.name)

        motors = listOf(fl, fr, bl, br,)

        motors.forEach {
            it.zeroPowerBehavior =
                if (brake) DcMotor.ZeroPowerBehavior.BRAKE else DcMotor.ZeroPowerBehavior.FLOAT

            it.targetPosition = 0

            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }


        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE
        fr.direction = DcMotorSimple.Direction.FORWARD
        br.direction = DcMotorSimple.Direction.REVERSE

        fl.setPower(0.7)
        bl.setPower(0.0)
        fr.setPower(0.0)
        br.setPower(0.7)

        sleep(10000)

        fl.setPower(0.0)
        bl.setPower(0.0)
        fr.setPower(0.0)
        br.setPower(0.0)

        sleep(2000)


        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE
        fr.direction = DcMotorSimple.Direction.FORWARD
        br.direction = DcMotorSimple.Direction.FORWARD

        drivetrain.move(y=12)

    }


}