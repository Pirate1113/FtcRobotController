package org.firstinspires.ftc.teamcode.testing;

import  com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightAngle {

    public final Limelight3A limelight;

    // Heights in inches
    public final double llHeight;
    public final double tagHeight;

    /**
     * @param hw HardwareMap from OpMode
     * @param limelightName // sure
     * @param shooterHeight // height of shooter from floor
     * @param tagHeight height of AprilTag from floor INCHES
     */
    public LimelightAngle(HardwareMap hw,
                          String limelightName,
                          double shooterHeight,
                          double tagHeight) {

        this.limelight = hw.get(Limelight3A.class, limelightName);
        this.limelight.pipelineSwitch(0);
        this.llHeight = shooterHeight;
        this.tagHeight = tagHeight;

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
    public double getDistanceInches(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();

        Pose3D pose = result.getBotpose();

        if (result == null || !result.isValid()) return 0.0;

        telemetry.addData("Botpos: ", pose.toString());

        double pitch = Math.toRadians(result.getTy());
        if (Math.abs(pitch) < 0.00872665) return 0.0;

        double verticalDiff = tagHeight - llHeight;

        return verticalDiff / Math.tan(pitch);
    }


    public void stop() {
        limelight.stop();
    }
}