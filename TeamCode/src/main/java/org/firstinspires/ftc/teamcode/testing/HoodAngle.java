package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodAngle {

    private final Servo hood;
    private final DcMotorEx flywheel;

    // needs measuring/tuning
    private static final double TARGET_HEIGHT  = 57.0; // inches (AprilTag)
    private static final double SHOOTER_HEIGHT = 12.0; // inches

    // tuning
    private static final double SERVO_INTERCEPT = 0.32;  // hood pos at 0°
    private static final double SERVO_SLOPE     = (double) 1 /360; // servo units per degree

    // flywheel
    private static final double BASE_RPM = 2200;
    private static final double RPM_PER_INCH = 67;

    public HoodAngle(HardwareMap hw) {
        hood = hw.get(Servo.class, "hoodServo");
        flywheel = hw.get(DcMotorEx.class, "shooter1");

        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


    public void aimFromDistance(double distanceInches) {

        if (distanceInches < 1) return;

        double hoodPos = hoodPositionFromDistance(distanceInches);
        hood.setPosition(hoodPos);

        double rpm = BASE_RPM + distanceInches * RPM_PER_INCH;
        flywheel.setVelocity(rpm);
    }

    /**
     * projectile motion
     */
    private double hoodPositionFromDistance(double distance) {

        double h = TARGET_HEIGHT - SHOOTER_HEIGHT;

        double angleRad =
                Math.atan((h + Math.sqrt(h * h + distance * distance)) / distance);

        double angleDeg = Math.toDegrees(angleRad);

        double servoPos =
                SERVO_INTERCEPT + SERVO_SLOPE * angleDeg;

        return clamp(servoPos);
    }

    public void stop() {
        flywheel.setPower(0);
    }

    private double clamp(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
