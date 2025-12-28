package org.firstinspires.ftc.teamcode.common.util;

public class AngleFunxuns {

    public static double wrapAngle0to2pi (double angle) {
        return ((angle % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
    }
}
