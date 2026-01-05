package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class OdomDistance {


    // Encoders
    private final DcMotorEx forwardEnc;
    private final DcMotorEx strafeEnc;

    // initial
    private double x = 0.0;
    private double y = 0.0;

    // initial
    private int lastForward = 0;
    private int lastStrafe = 0;

    // constants
    private static final double TICKS_PER_REV = 2000; // goBILDA
    private static final double WHEEL_RADIUS = 0.629921; // inches (32mm diameter)
    private static final double GEAR_RATIO = 1.0; // should just be 1:1

    public static final double HOOP_X = -30.0; // inches  BOTH THESE VALUES SHOULD BE TWEAKED DURING TESTING
    public static final double HOOP_Y = 72.0; // inches

    public OdomDistance(DcMotorEx forwardEnc, DcMotorEx strafeEnc) {
        this.forwardEnc = forwardEnc;
        this.strafeEnc = strafeEnc;
        resetEncoders();
    }

    private void resetEncoders() {
        lastForward = forwardEnc.getCurrentPosition();
        lastStrafe = strafeEnc.getCurrentPosition();
    }

    private double ticksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    /**
     * need to call odom.update every loop
     * @param headingRadians robot heading from AprilTag
     */

    public void update(double headingRadians) {

        int currForward = forwardEnc.getCurrentPosition();
        int currStrafe = strafeEnc.getCurrentPosition();

        int dForward = currForward - lastForward;
        int dStrafe = currStrafe - lastStrafe;

        lastForward = currForward;
        lastStrafe = currStrafe;

        double forwardInches = ticksToInches(dForward);
        double strafeInches = ticksToInches(dStrafe);

        // makes field-centric??
        double sin = Math.sin(headingRadians);
        double cos = Math.cos(headingRadians);

        x += forwardInches * cos - strafeInches * sin;
        y += forwardInches * sin + strafeInches * cos;
    }

    //
    public double getX() { return x; }
    public double getY() { return y; }

    public void setPose(double x, double y) {
        this.x = x;
        this.y = y;
        resetEncoders();
    }

}



