package org.firstinspires.ftc.teamcode.testing;

import static java.lang.Math.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HoodAngle {
    private final Servo hood;
    public DcMotorEx flywheel;

    //constants

    // heights in inches
    public static double shooterHeight = 12.0;
    public static double tagHeight = 29.5;


    //hood tuning

    private static final double SERVO_DEG_PER_HOOD = 13;

    //
    public static final double BASE_RPM = 2200;
    public static final double RPM_PER_INCH = 67; //this is basically fake idk if we need this but tune 67


    public HoodAngle(HardwareMap hw,
                     double LLHeight,
                     double tagHeight) {

        hood = hw.get(Servo.class, "hoodServo");
        flywheel = hw.get(DcMotorEx.class, "shooter1");

        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


    public void aimFromDistance(double distanceInches) {
        if (distanceInches < 1.0) return;

        double hoodPos = hoodPositionFromDistance(distanceInches);
        hood.setPosition(hoodPos);

        double rpm = BASE_RPM + distanceInches * RPM_PER_INCH;
        flywheel.setVelocity(rpm);
    }

    public void aimFromLimelight(LimelightAngle limelight) {
        if (!limelight.hasTarget()) return;

        double distance = limelight.getDistanceInches();
        aimFromDistance(distance);
    }


    public void stop() {
        flywheel.setPower(0);
    }

    public double getFlywheelRpm() {
        double ticksPerSec = flywheel.getVelocity();
        return ticksPerSec * 60.0 / 28.0;
    }
    public double radiusMeters = 0.0376565;

    public double getInitialVelocity(double radiusMeters) {
        double rpm = getFlywheelRpm();
        double v =  rpm * 2.0 * Math.PI * radiusMeters / 60.0; // m/s
        return v;
    }

    // projectile math
    public double hoodPositionFromDistance(double distance) {

        double verticalDiff = tagHeight - shooterHeight;

        double angleRad = Math.atan(verticalDiff / distance);

        double projectileAngle = (double) 1 /2*asin((distance*9.81)/getInitialVelocity(0.0376565));

        double angleDeg = Math.toDegrees(angleRad);

        double servoPos = angleDeg * SERVO_DEG_PER_HOOD/255;

        return Range.clip(servoPos, 0, 1.0);
    }


}