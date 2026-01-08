package org.firstinspires.ftc.teamcode.testing;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

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
        return result != null
                && result.isValid()
                && !result.getFiducialResults().isEmpty();
    }

    public double getYaw() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0.0;

        return result.getFiducialResults().get(0).tx;
    }

    public double getPitch() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0.0;

        return result.getFiducialResults().get(0).ty;
    }

    public double getDistance() {
        double pitchRad = Math.toRadians(getPitch());
        double verticalDiff = SHOOTER_HEIGHT + TAG_OFFSET;
        return verticalDiff / Math.tan(pitchRad);
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