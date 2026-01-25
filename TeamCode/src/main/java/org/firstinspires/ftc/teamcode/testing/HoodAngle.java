package org.firstinspires.ftc.teamcode.testing;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

<<<<<<< HEAD
/**
 * Controls hood servo and flywheel speed for auto-aim shooting.
 */
=======
import org.firstinspires.ftc.robotcore.external.Telemetry;

>>>>>>> 58e1ccaacf0918166dce8467964b78cd2725b165
public class HoodAngle {

    private final Servo hood;
    DcMotorEx shooter1;
    DcMotorEx shooter2;


    public double RPM;

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
        shooter1 = hw.get(DcMotorEx.class, "shooter1");
        shooter2 = hw.get(DcMotorEx.class, "shooter2");

        shooter1.setDirection(DcMotor.Direction.REVERSE);

        shooter2.setDirection(DcMotor.Direction.FORWARD);

        this.shooterHeightInches = shooterHeightInches;
        this.tagHeightInches = tagHeightInches;

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * aim hood based on distance and run flywheel at constant 6000 RPM.
     *
     * @param distanceInches horizontal distance from SHOOTER to tag (inches)
     */
    public void aimFromDistance(double distanceInches) {
        if (distanceInches < 1.0) return; // ignore bad/zero readings

        double hoodPos = hoodPositionFromDistance(distanceInches);
        hood.setPosition(hoodPos);

<<<<<<< HEAD
        setFlywheelRpm(FLYWHEEL_RPM);
    }

    /** aim using LimelightAngle directly */
    public void aimFromLimelight(LimelightAngle limelight,
                                 double shooterToCameraOffsetInches) {
        if (!limelight.hasTarget()) return;

        // Limelight distance is camera-to-tag. Convert to shooter-to-tag if needed.
        double distCamera = limelight.getDistanceInches();
        if (distCamera <= 0.0) return;

        double distShooter = distCamera + shooterToCameraOffsetInches;
        aimFromDistance(distShooter);
    }

=======
        double rpm = BASE_RPM + distanceInches * RPM_PER_INCH;
        shooter1.setVelocity(rpm);
        shooter2.setVelocity(rpm);
    }

    public void setHoodPos (double pos) {
        hood.setPosition(pos);

    }

    /** Convenience method: use LimelightAngle directly */
    public void aimFromLimelight(LimelightAngle limelight, Telemetry telemetry) {
        if (!limelight.hasTarget()) return;

        double distance = limelight.getDistanceInches(telemetry);
        aimFromDistance(distance);
    }

    public void getTelemetry (Telemetry telemetry) {
        telemetry.addData("shooter1: ", shooter1.getVelocity()*60/28);
        telemetry.addData("shooter2: ", shooter2.getVelocity()*60/28);
    }

    /** Stop flywheel (hood stays where it is) */
>>>>>>> 58e1ccaacf0918166dce8467964b78cd2725b165
    public void stop() {
        shooter1.setPower(0);
        shooter2.setPower(0);
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

        return MathUtils.clamp(servoPos, 0.0, 1.0);
    }

<<<<<<< HEAD
    private double clamp(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }

    /** Convert desired RPM to encoder ticks/sec and send to motor */
    private void setFlywheelRpm(double rpm) {
        // ticks/sec = rpm * TICKS_PER_REV * gearRatio / 60
        double ticksPerSec = rpm * TICKS_PER_REV * GEAR_RATIO / 60.0;

        flywheel.setVelocity(ticksPerSec);
    }
=======
>>>>>>> 58e1ccaacf0918166dce8467964b78cd2725b165
}