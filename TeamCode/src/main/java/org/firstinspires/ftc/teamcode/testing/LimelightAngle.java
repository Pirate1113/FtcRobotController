import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.network.NetworkTable;
import org.firstinspires.ftc.robotcore.external.network.NetworkTableEntry;
import org.firstinspires.ftc.robotcore.external.network.NetworkTableInstance;

public class LimelightAngle {

    private final NetworkTable limelight;   // Limelight NetworkTable
    private final double SHOOTER_HEIGHT;    // shooter exit height in inches
    private final double TAG_OFFSET;        // how far below the hoop the AprilTag is
    private final double ROBOT_X_OFFSET;    // distance from robot center to shooter

    public LimelightAngle(double shooterHeight, double tagOffset, double robotXOffset) {
        this.SHOOTER_HEIGHT = shooterHeight;
        this.TAG_OFFSET = tagOffset;
        this.ROBOT_X_OFFSET = robotXOffset;

        // Get Limelight NetworkTable
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /** Horizontal angle as yaw in degrees */
    public double getYaw() {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    /** Vertical angle as pitch in degrees */
    public double getPitch() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    /** Returns true if Limelight sees the target */
    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /** Estimate horizontal distance from shooter to target using pitch */
    public double getDistance() {
        double pitchDeg = getPitch();
        double pitchRad = Math.toRadians(pitchDeg);

        double verticalDiff = TAG_OFFSET + SHOOTER_HEIGHT;
        return verticalDiff / Math.tan(pitchRad); // horizontal distance in inches
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