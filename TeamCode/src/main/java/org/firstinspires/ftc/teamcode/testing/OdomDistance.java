package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class OdomDistance {

    // encoders
    private final DcMotorEx forwardEnc;
    private final DcMotorEx strafeEnc;

    // pose (field-centric)
    private double x = 0.0; // inches
    private double y = 0.0; // inches


    private int lastForward = 0;
    private int lastStrafe = 0;

    // constants TUNE
    private static final double TICKS_PER_REV = 2000;
    private static final double WHEEL_RADIUS = 0.629921; // inches (32mm)
    private static final double GEAR_RATIO = 1.0;

    // ===== FIELD TARGET (HOOP) =====
    public static final double HOOP_X = -30.0; // inches
    public static final double HOOP_Y = 72.0;  // inches

    // ===== CONSTRUCTOR =====
    public OdomDistance(DcMotorEx forwardEnc, DcMotorEx strafeEnc) {
        this.forwardEnc = forwardEnc;
        this.strafeEnc = strafeEnc;
        resetEncoders();
    }

    // ===== CORE ODOMETRY =====
    public void update(double headingRadians) {

        int currForward = forwardEnc.getCurrentPosition();
        int currStrafe = strafeEnc.getCurrentPosition();

        int dForward = currForward - lastForward;
        int dStrafe  = currStrafe  - lastStrafe;

        lastForward = currForward;
        lastStrafe  = currStrafe;

        double forwardInches = ticksToInches(dForward);
        double strafeInches  = ticksToInches(dStrafe);

        // Robot-centric → field-centric
        double sin = Math.sin(headingRadians);
        double cos = Math.cos(headingRadians);

        x += forwardInches * cos - strafeInches * sin;
        y += forwardInches * sin + strafeInches * cos;
    }

    // ===== GEOMETRY HELPERS =====

    /** Distance from robot to hoop (inches) */
    public double getDistanceToHoop() {
        double dx = HOOP_X - x;
        double dy = HOOP_Y - y;
        return Math.hypot(dx, dy);
    }

    /** Field-centric angle from robot to hoop */
    public double getAngleToHoopField() {
        double dx = HOOP_X - x;
        double dy = HOOP_Y - y;
        return Math.atan2(dy, dx);
    }

    /** Robot-relative yaw error (what to turn) */
    public double getYawErrorToHoop(double headingRadians) {
        double angleToHoop = getAngleToHoopField();
        double error = angleToHoop - headingRadians;

        // Normalize to [-pi, pi]
        return Math.atan2(Math.sin(error), Math.cos(error));
    }

    // ===== UTIL =====
    private double ticksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    private void resetEncoders() {
        lastForward = forwardEnc.getCurrentPosition();
        lastStrafe  = strafeEnc.getCurrentPosition();
    }

    public void setPose(double x, double y) {
        this.x = x;
        this.y = y;
        resetEncoders();
    }

    // ===== GETTERS =====
    public double getX() { return x; }
    public double getY() { return y; }
}


