package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightAngle {

    public final Limelight3A limelight;

    // Heights in inches
    public static double LLHeight;
    public static double tagHeight;

    /**
     * @param hw HardwareMap from OpMode
     * @param limelightName // sure
     * @param LLHeight // height of shooter from floor
     * @param tagHeight height of AprilTag from floor INCHES
     */
    public LimelightAngle(HardwareMap hw,
                          String limelightName,
                          double LLHeight,
                          double tagHeight) {

        this.limelight = hw.get(Limelight3A.class, limelightName);
        this.LLHeight = LLHeight;
        this.tagHeight = tagHeight;

        limelight.start();
    }

    public void pipelineSwitch (int index) {
        limelight.pipelineSwitch(index);
    }

    /** @return true if Limelight currently sees a valid target */
    public boolean hasTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    /** @return horizontal yaw offset in degrees */
    public double getYaw() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTx() : 0.0;
    }

    /** @return vertical pitch offset in degrees */
    public double getPitch() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTy() : 0.0;
    }

    /**
     *
     * tan(theta) = (tagHeight - shooterHeight) / distance
     *
     * @return distance in inches
     */
    public double getDistanceInches() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0.0;

        double pitchDeg = result.getTy();

        double pitchRad = Math.toRadians(pitchDeg);
        double verticalDiff = tagHeight - LLHeight;

        return verticalDiff / Math.tan(pitchRad);
    }


    public void stop() {
        limelight.stop();
    }
}