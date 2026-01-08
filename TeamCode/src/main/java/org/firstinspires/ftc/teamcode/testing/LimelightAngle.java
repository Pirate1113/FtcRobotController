package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class LimelightAngle {

    private final Limelight3A limelight;
    private final double SHOOTER_HEIGHT;
    private final double TAG_OFFSET;

    public LimelightAngle(Limelight3A limelight,
                          double shooterHeight,
                          double tagOffset) {
        this.limelight = limelight;
        this.SHOOTER_HEIGHT = shooterHeight;
        this.TAG_OFFSET = tagOffset;
    }

    public boolean hasTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public double getYaw() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTx() : 0.0;
    }

    public double getPitch() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTy() : 0.0;
    }

    public double getDistance() {
        double pitchRad = Math.toRadians(getPitch());
        return (TAG_OFFSET + SHOOTER_HEIGHT) / Math.tan(pitchRad);
    }
}


/**
     * field-centric distance using odometry got rid
     */
//    public double getFieldDistance(double hoopX, double hoopY) {
//        double dx = hoopX - (odom.getX() + ROBOT_X_OFFSET);
//        double dy = hoopY - odom.getY();
//        return Math.hypot(dx, dy);
//    }
//}