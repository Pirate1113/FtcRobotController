package org.firstinspires.ftc.teamcode.testing;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HoodAngle {

    private final Servo hood;
    DcMotorEx shooter1;
    DcMotorEx shooter2;


    public double RPM;

    //constants

    // heights in inches
    public static double SHOOTER_HEIGHT = 12.0;
    public static double TAG_HEIGHT = 37.0;

    //hood tuning

    // servo position at 0 degrees idk how to figure
    public static final double SERVO_INTERCEPT = 0.32;

    // should work but kinda ignoring gear ratio
    public static final double SERVO_SLOPE = 1.0 / 360.0;



    //
    private static final double BASE_RPM = 2200;
    private static final double RPM_PER_INCH = 67; //this is basically fake idk if we need this but tune 67


    public HoodAngle(HardwareMap hw,
                     double shooterHeight,
                     double tagHeight) {

        hood = hw.get(Servo.class, "hoodServo");
        shooter1 = hw.get(DcMotorEx.class, "shooter1");
        shooter2 = hw.get(DcMotorEx.class, "shooter2");

        shooter1.setDirection(DcMotor.Direction.REVERSE);

        shooter2.setDirection(DcMotor.Direction.FORWARD);

        SHOOTER_HEIGHT = shooterHeight;
        TAG_HEIGHT = tagHeight;

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    /** Aim hood + flywheel directly from Limelight distance */
    public void aimFromDistance(double distanceInches) {
        if (distanceInches < 1.0) return;

        double hoodPos = hoodPositionFromDistance(distanceInches);
        hood.setPosition(hoodPos);

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
    public void stop() {
        shooter1.setPower(0);
        shooter2.setPower(0);
    }


    // projectile math
    private double hoodPositionFromDistance(double distance) {

        double verticalDiff = TAG_HEIGHT - SHOOTER_HEIGHT;

        double angleRad =
                Math.atan((verticalDiff + Math.sqrt(
                        verticalDiff * verticalDiff + distance * distance))
                        / distance);

        double angleDeg = Math.toDegrees(angleRad);

        double servoPos =
                SERVO_INTERCEPT + SERVO_SLOPE * angleDeg;

        return MathUtils.clamp(servoPos, 0.0, 1.0);
    }

}