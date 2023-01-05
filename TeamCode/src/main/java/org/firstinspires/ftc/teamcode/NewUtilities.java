package org.firstinspires.ftc.teamcode;

import java.lang.Math;

public class NewUtilities {

    public double MILLISECONDS_PER_SECOND = 1.0e3;
    public double MILLIMETERS_PER_INCH = 25.4;
    public double NANOSECONDS_PER_SECONDS = 1.0e9;
    public double DEGREES_PER_ROTATION = 360.0;

    public static double square(Double x) {
        return x * x;
    }

    public static double toRadian(Double x) {
        return x * Math.PI / 180.0;
    }



}
