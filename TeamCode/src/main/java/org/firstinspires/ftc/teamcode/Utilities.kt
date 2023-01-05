package org.firstinspires.ftc.teamcode

import kotlin.math.*

const val MILLISECONDS_PER_SECOND = 1.0e3
const val NANOSECONDS_PER_SECOND = 1.0e9
const val MILLIMETERS_PER_INCH = 25.4
const val DEGREES_PER_ROTATION = 360.0

fun Double.squared() = this * this
fun Double.toRadians() = this * PI / 180.0

typealias TwoDimensionalPoint = TwoDimensionalVector

data class TwoDimensionalVector(val x: Double = 0.0, val y: Double = 0.0) {
    constructor(x: Number, y: Number) : this(x.toDouble(), y.toDouble())

    val magnitude = hypot(x, y)

    private val heading = atan2(y, x)

    fun rotatedAboutOrigin(displacementAngle: Double): TwoDimensionalVector {
        val heading = this.heading + displacementAngle.toRadians()
        return TwoDimensionalVector(
            magnitude * cos(heading),
            magnitude * sin(heading)
        )
    }

    infix fun dot(other: TwoDimensionalVector): Double {
        return x * other.x + y * other.y
    }

    operator fun plus(other: TwoDimensionalVector): TwoDimensionalVector {
        return TwoDimensionalVector(
            x + other.x,
            y + other.y
        )
    }

    operator fun minus(other: TwoDimensionalVector): TwoDimensionalVector {
        return TwoDimensionalVector(
            x - other.x,
            y - other.y
        )
    }

    operator fun times(scalar: Double): TwoDimensionalVector {
        return TwoDimensionalVector(
            x * scalar,
            y * scalar
        )
    }

    operator fun div(scalar: Double): TwoDimensionalVector {
        return TwoDimensionalVector(
            x / scalar,
            y / scalar
        )
    }
}