package org.firstinspires.ftc.teamcode.testing;

import static java.lang.Math.*;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HoodAngle {

    private final Servo hood;
    public DcMotorEx flywheel;

    //constants

    // heights in inches
    public static double shooterHeight = 12.0;
    public static double tagHeight = 29.5;


    //hood tuning

    private static final double SERVO_DEG_PER_HOOD = 8.125;

    //

    public HoodAngle(HardwareMap hw,
                     double LLHeight,
                     double tagHeight) {

        hood = hw.get(Servo.class, "hoodServo");
        flywheel = hw.get(DcMotorEx.class, "shooter1");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


    public void aimFromDistance(double distanceInches, Telemetry telemetry) {
        if (distanceInches < 1.0) return;
        flywheel.setVelocity(3500);

        telemetry.addData("hoodPos: ", hoodPositionFromDistance(distanceInches, telemetry));

        if (Double.isNaN(hoodPositionFromDistance(distanceInches, telemetry)) ){
            hood.setPosition(0);
        } else {
            hood.setPosition(hoodPositionFromDistance(distanceInches, telemetry));
        }

    }


    public void stop() {
        flywheel.setPower(0);
    }

    public double getFlywheelRpm() {
        double ticksPerSec = flywheel.getVelocity();
        return ticksPerSec * 60.0 / 28.0;
    }

    public double getInitialVelocity(double radiusInches) {
        return getFlywheelRpm() * 2.0 * Math.PI * radiusInches / 60.0; // in/s;
    }

    // projectile math fixed

    public double getProjectileAngle(double distance, Telemetry telemetry) {
        double g = 386.4; // in/s^2
        double y = tagHeight - shooterHeight; // y val
        double x = distance;

        double v0 = getInitialVelocity(1.5);
        telemetry.addData("velocity: ", v0);

        double a = (g*Math.pow(x, 2))/(2*Math.pow(v0, 2));
        double b = -x;
        double c = y + (g*Math.pow(x, 2))/(2*Math.pow(v0, 2));

        double discriminant = b*b - 4*a*c;
        telemetry.addData("discrimination: ", discriminant);
        double tanTheta = (-b + Math.sqrt(discriminant)) / (2*a);
        // make the sign in front of the discriminant a positive for the high angle
        double projectileAngleRad = Math.atan(tanTheta);

        return Math.toDegrees(projectileAngleRad);
    }

    public double hoodPositionFromDistance(double distance, Telemetry telemetry) {
        double angleDeg = getProjectileAngle(distance, telemetry);
        double servoPos = Math.abs(Range.clip((angleDeg * SERVO_DEG_PER_HOOD / 255.0), 0.0, 1.0));

        telemetry.addData("what the hood is supposed to be at: ", angleDeg);
        telemetry.addData("Servoting: ", servoPos);

        return servoPos;
    }


}