package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Controls hood servo and flywheel speed for auto-aim shooting.
 */
public class HoodAngle {

    private final Servo hood;
    private final DcMotorEx flywheel;

    // Shooter & target geometry (INCHES)
    private final double shooterHeightInches;
    private final double tagHeightInches;


    // servo position that corresponds to 0 degrees
    public static final double SERVO_INTERCEPT = 0.32;
    // servo position change per degree of hood angle we might need to tune for gear ratio
    public static final double SERVO_SLOPE = 1.0 / 360.0;

    // Flywheel speed (constant 6000 RPM as requested)
    private static final double FLYWHEEL_RPM = 6000.0;

    // motor parameters (for velocity in ticks/sec)
    private static final int TICKS_PER_REV = 28; // GoBILDA 5202/5203 base encoder
    private static final double GEAR_RATIO = 1.0; // tune if necessary?!?

    public HoodAngle(HardwareMap hw,
                     double shooterHeightInches,
                     double tagHeightInches) {

        hood = hw.get(Servo.class, "hoodServo");
        flywheel = hw.get(DcMotorEx.class, "shooter1");

        this.shooterHeightInches = shooterHeightInches;
        this.tagHeightInches = tagHeightInches;

        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * aim hood based on distance and run flywheel at constant 6000 RPM
     *
     */
    public void aimFromDistance(double distanceInches) {
        if (distanceInches < 1.0) return; // ignore bad/zero readings

        double hoodPos = hoodPositionFromDistance(distanceInches);
        hood.setPosition(hoodPos);

        setFlywheelRpm(FLYWHEEL_RPM);
    }

    /** aim using LimelightAngle directly */
    public void aimFromLimelight(LimelightAngle limelight,
                                 double shooterToCameraOffsetInches) {
        if (!limelight.hasTarget()) return;

        double distCamera = limelight.getDistanceInches();
        if (distCamera <= 0.0) return;

        double distShooter = distCamera + shooterToCameraOffsetInches;
        aimFromDistance(distShooter);
    }

    public void stop() {
        flywheel.setPower(0);
    }


    private double hoodPositionFromDistance(double distanceInches) {

        double verticalDiff = tagHeightInches - shooterHeightInches;

        // atan( (h + sqrt(h^2 + d^2)) / d )
        double angleRad =
                Math.atan((verticalDiff + Math.sqrt(
                        verticalDiff * verticalDiff + distanceInches * distanceInches))
                        / distanceInches);

        double angleDeg = Math.toDegrees(angleRad);

        double servoPos =
                SERVO_INTERCEPT + SERVO_SLOPE * angleDeg;

        return clamp(servoPos);
    }

    private double clamp(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }

    /** convert desired RPM to encoder ticks/sec and send to motor */
    private void setFlywheelRpm(double rpm) {
        // ticks/sec = rpm * TICKS_PER_REV * gearRatio / 60
        double ticksPerSec = rpm * TICKS_PER_REV * GEAR_RATIO / 60.0;

        flywheel.setVelocity(ticksPerSec);
    }
}