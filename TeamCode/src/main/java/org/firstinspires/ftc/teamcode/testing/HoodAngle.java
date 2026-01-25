package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodAngle {

    private final Servo hood;
    private final DcMotorEx flywheel;

    //constants

    // heights in inches
    public static double SHOOTER_HEIGHT = 12.0;
    public static double TAG_HEIGHT = 37.0;

    //hood tuning

    // servo position at 0 degrees idk how to figure
    public static final double SERVO_INTERCEPT = 0.32;

    // should work but kinda ignoring gear ratio
    public static final double SERVO_SLOPE = 1.0 / 360.0;



    //
    private static final double BASE_RPM = 2200;
    private static final double RPM_PER_INCH = 67; //this is basically fake idk if we need this but tune 67


    public HoodAngle(HardwareMap hw,
                     double shooterHeight,
                     double tagHeight) {

        hood = hw.get(Servo.class, "hoodServo");
        flywheel = hw.get(DcMotorEx.class, "shooter1");

        SHOOTER_HEIGHT = shooterHeight;
        TAG_HEIGHT = tagHeight;

        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


    /** Aim hood + flywheel directly from Limelight distance */
    public void aimFromDistance(double distanceInches) {
        if (distanceInches < 1.0) return;

        double hoodPos = hoodPositionFromDistance(distanceInches);
        hood.setPosition(hoodPos);

        double rpm = BASE_RPM + distanceInches * RPM_PER_INCH;
        flywheel.setVelocity(rpm);
    }

    /** Convenience method: use LimelightAngle directly */
    public void aimFromLimelight(LimelightAngle limelight) {
        if (!limelight.hasTarget()) return;

        double distance = limelight.getDistanceInches();
        aimFromDistance(distance);
    }

    /** Stop flywheel (hood stays where it is) */
    public void stop() {
        flywheel.setPower(0);
    }


    // projectile math
    private double hoodPositionFromDistance(double distance) {

        double verticalDiff = TAG_HEIGHT - SHOOTER_HEIGHT;

        double angleRad =
                Math.atan((verticalDiff + Math.sqrt(
                        verticalDiff * verticalDiff + distance * distance))
                        / distance);

        double angleDeg = Math.toDegrees(angleRad);

        double servoPos =
                SERVO_INTERCEPT + SERVO_SLOPE * angleDeg;

        return clamp(servoPos);
    }

    private double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}