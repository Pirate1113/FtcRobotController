package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LimelightAngle {

    public final Limelight3A limelight;

    // Camera geometry (INCHES, DEGREES)
    // TODO: MEASURE AND TUNE THESE on your robot.
    private final double cameraHeightInches;   // height of Limelight lens from floor
    private final double cameraPitchDeg;       // upward tilt of Limelight relative to horizontal

    private final double tagHeightInches;      // AprilTag height from floor

    /**
     * @param hw HardwareMap from OpMode
     * @param limelightName device name in Robot Configuration
     * @param cameraHeightInches Limelight lens height from floor
     * @param cameraPitchDeg Limelight mounting angle (deg, positive looking up)
     * @param tagHeightInches Tag height from floor
     */
    public LimelightAngle(HardwareMap hw,
                          String limelightName,
                          double cameraHeightInches,
                          double cameraPitchDeg,
                          double tagHeightInches) {

        this.limelight = hw.get(Limelight3A.class, limelightName);

        this.cameraHeightInches = cameraHeightInches;
        this.cameraPitchDeg = cameraPitchDeg;
        this.tagHeightInches = tagHeightInches;

        limelight.start();
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

    /** @return vertical pitch offset in degrees (target relative to camera crosshair) */
    public double getPitch() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getTy() : 0.0;
    }


    public double getDistanceInches() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0.0;

        double tyDeg = result.getTy();
        double totalAngleDeg = cameraPitchDeg + tyDeg;

        // avoid division by very small angles
        if (Math.abs(totalAngleDeg) < 0.5) { // 0.5 degree threshold we might need to tune
            return 0.0;
        }

        double totalAngleRad = Math.toRadians(totalAngleDeg);
        double verticalDiff = tagHeightInches - cameraHeightInches;

        if (Math.abs(Math.tan(totalAngleRad)) < 1e-6) {
            return 0.0;
        }

        double distance = verticalDiff / Math.tan(totalAngleRad);


        if (distance < 0.0) distance = 0.0;
        return distance;
    }

    public void stop() {
        limelight.stop();
    }
}