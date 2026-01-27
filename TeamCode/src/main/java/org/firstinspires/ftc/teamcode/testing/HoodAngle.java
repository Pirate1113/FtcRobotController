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

        flywheel.setVelocity(3500);
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
    public double radiusInches = 1.5;

    public double getInitialVelocity(double radiusInches) {
        double rpm = getFlywheelRpm();
        double v =  rpm * 2.0 * Math.PI * radiusInches / 60.0; // in/s
        return v;
    }

    // projectile math fixed

    public double getProjectileAngle(double distance) {
        double g = 386.4; // in/s^2
        double y = tagHeight - shooterHeight; // y val
        double x = distance;

        double v0 = getInitialVelocity(1.5);

        double a = (g*Math.pow(x, 2))/(2*Math.pow(v0, 2));
        double b = -x;
        double c = y + (g*Math.pow(x, 2))/(2*Math.pow(v0, 2));

        double discriminant = b*b - 4*a*c;
        double tanTheta = (-b - Math.sqrt(discriminant)) / (2*a);
        // make the sign in front of the discriminant a positive for the high angle
        double projectileAngleRad = Math.atan(tanTheta);
        double angleDeg = Math.toDegrees(projectileAngleRad);
        return Math.toDegrees(projectileAngleRad);
    }

    public double hoodPositionFromDistance(double distance) {
        double angleDeg = getProjectileAngle(distance);
        double servoPos = angleDeg * SERVO_DEG_PER_HOOD / 255.0;
        return Range.clip(servoPos, 0.0, 1.0);
    }


}