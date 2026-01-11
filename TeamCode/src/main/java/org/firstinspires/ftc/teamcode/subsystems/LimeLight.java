//package org.firstinspires.ftc.teamcode.testing;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//
//import dev.nextftc.core.subsystems.Subsystem;
//
//public class Limelight implements Subsystem {
//
//    private final Limelight3A limelight;
//    private final double SHOOTER_HEIGHT;
//    private final double TAG_OFFSET;
//
//    public LimelightAngle(Limelight3A limelight,
//                          double shooterHeight,
//                          double tagOffset) {
//        this.limelight = limelight;
//        this.SHOOTER_HEIGHT = shooterHeight;
//        this.TAG_OFFSET = tagOffset;
//    }
//
//    /** Returns true if Limelight sees a target */
//    public boolean hasTarget() {
//        LLResult result = limelight.getLatestResult();
//        return result != null && result.isValid();
//    }
//
//    /** Horizontal angle as yaw in degrees */
//    public double getYaw() {
//        LLResult result = limelight.getLatestResult();
//        if(result != null && result.isValid()) {
//            return result.getTx(); // degrees
//        }
//        return 0.0;
//    }
//
//    /** Vertical angle as pitch in degrees */
//    public double getPitch() {
//        LLResult result = limelight.getLatestResult();
//        if(result != null && result.isValid()) {
//            return result.getTy(); // degrees
//        }
//        return 0.0;
//    }
//
//    /** Horizontal distance from shooter to hoop using pitch */
//    public double getDistance() {
//        double pitchDeg = getPitch();
//        if(pitchDeg == 0) return 0; // avoid division by zero
//        double pitchRad = Math.toRadians(pitchDeg);
//        double verticalDiff = TAG_OFFSET + SHOOTER_HEIGHT;
//        return verticalDiff / Math.tan(pitchRad);
//    }
//}
//
//
///**
// * field-centric distance using odometry got rid
// */
////    public double getFieldDistance(double hoopX, double hoopY) {
////        double dx = hoopX - (odom.getX() + ROBOT_X_OFFSET);
////        double dy = hoopY - odom.getY();
////        return Math.hypot(dx, dy);
////    }
////}