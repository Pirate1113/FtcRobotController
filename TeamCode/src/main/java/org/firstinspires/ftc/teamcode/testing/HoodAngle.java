package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.hardware.impl.ServoEx;
import com.qualcomm.robotcore.util.Range;

public class HoodAngle {
    private final ServoEx hood;

    // heights in inches
    public static double shooterHeight = 12.0;
    public static double tagHeight = 37.0;

    private static final double SERVO_DEG_PER_HOOD = 8.125;

    public HoodAngle(HardwareMap hw,
                     double LLHeight,
                     double tagHeight) {

        hood = new ServoEx("hoodServo", 0.04);
    }

    public void aimFromDistance(double distanceInches) {
        if (distanceInches < 1.0) return;

        double hoodPos = hoodPositionFromDistance(distanceInches);
        hood.setPosition(hoodPos);

    }

    public void aimFromLimelight(LimelightAngle limelight) {
        if (!limelight.hasTarget()) return;

        double distance = limelight.getDistanceInches();
        aimFromDistance(distance);
    }

    // projectile math
    private double hoodPositionFromDistance(double distance) {

        double verticalDiff = tagHeight - shooterHeight;

        double angleRad =
                Math.atan((verticalDiff + Math.sqrt(
                        verticalDiff * verticalDiff + distance * distance))
                        / distance);

        double angleDeg = Math.toDegrees(angleRad);

        double servoPos = angleDeg * SERVO_DEG_PER_HOOD/255;

        return Range.clip(servoPos, 0, 1.0);
    }

}