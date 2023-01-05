package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.gamepad3
import org.firstinspires.ftc.teamcode.hardwareMap
import org.firstinspires.ftc.teamcode.telemetry

class Cap : Mechanism {
    private lateinit var horizontal: Servo
    private lateinit var vertical: Servo
    private lateinit var extend: CRServo

    override fun initialize() {
        horizontal = hardwareMap.get(Servo::class.java, ::horizontal.name)
        vertical = hardwareMap.get(Servo::class.java, ::vertical.name)
        extend = hardwareMap.get(CRServo::class.java, ::extend.name)

        vertical.position = 0.26722
        horizontal.position = 0.68
    }

    override fun update() {
        when {
            gamepad3.dpad_right -> horizontal.position += 0.0001
            gamepad3.dpad_left -> horizontal.position -= 0.0001
        }
//        if (horizontal.position < 0.56) horizontal.position = 0.56
        when {
            gamepad3.dpad_up -> vertical.position += 0.0001
            gamepad3.dpad_down -> vertical.position -= 0.0001
        }
//        if (vertical.position < 0.393) vertical.position = 0.393
//        if (vertical.position > 0.428) vertical.position = 0.393
        extend.power = when {
            gamepad3.right_bumper -> 0.95
            gamepad3.left_bumper -> -0.95
            else -> 0.0
        }
    }

    override fun telemetry() {
        telemetry.addData("horizontal currentPosition", horizontal.position)
        telemetry.addData("vertical position", vertical.position)
        telemetry.addData("extend power", extend.power)
    }
}