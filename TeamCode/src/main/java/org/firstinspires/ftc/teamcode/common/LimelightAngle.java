package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.teamcode.testing.OdomDistance;

public class LimelightAngle {
        private final OdomDistance odom;
        private final double HOOP_HEIGHT;       // hoop center height in inches
        private final double SHOOTER_HEIGHT;    // shooter exit height in inches
        private final double TAG_OFFSET;        // vertical offset from AprilTag to hoop center
        private final double ROBOT_X_OFFSET;    // distance from robot center to shooter

        public LimelightAngle(OdomDistance odom, double hoopHeight, double shooterHeight, double tagOffset, double robotXOffset) {
            this.odom = odom;
            this.HOOP_HEIGHT = hoopHeight;
            this.SHOOTER_HEIGHT = shooterHeight;
            this.TAG_OFFSET = tagOffset;
            this.ROBOT_X_OFFSET = robotXOffset;
        }

        /**
         * Get hood pitch (vertical angle) in radians
         */
        public double getHoodPitch() {
            // Distance from shooter to hoop in horizontal plane
            double dx = OdomDistance.HOOP_X - (odom.getX() + ROBOT_X_OFFSET);
            double dy = OdomDistance.HOOP_Y - odom.getY();
            double horizontalDistance = Math.hypot(dx, dy);

            // Vertical difference from shooter to hoop center
            double verticalDiff = HOOP_HEIGHT - SHOOTER_HEIGHT + TAG_OFFSET;

            return Math.atan2(verticalDiff, horizontalDistance);
        }

        /**
         * Get robot yaw (horizontal angle) to face hoop
         * Returns radians relative to robot's current heading (0 = forward)
         */
        public double getRobotYaw() {
            double dx = OdomDistance.HOOP_X - (odom.getX() + ROBOT_X_OFFSET);
            double dy = OdomDistance.HOOP_Y - odom.getY();

            double targetAngle = Math.atan2(dy, dx);

            return targetAngle;
        }

        /**
         * Distance to hoop
         */
        public double getDistance() {
            double dx = OdomDistance.HOOP_X - (odom.getX() + ROBOT_X_OFFSET);
            double dy = OdomDistance.HOOP_Y - odom.getY();
            return Math.hypot(dx, dy);
        }
    }