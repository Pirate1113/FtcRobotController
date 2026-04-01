package org.firstinspires.ftc.teamcode.common.util;

public class AngleFunxuns {

    public static double wrapAngle0to2pi (double angle) {
        return ((angle % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
    }

    public static double normRads (double radians) {
        final double TWO_PI = 2 * Math.PI;
        double normalized = (radians + Math.PI) % TWO_PI;
        if (normalized < 0) normalized += TWO_PI;
        return normalized - Math.PI;
    }
}
