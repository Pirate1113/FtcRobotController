package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.hardware.impl.ServoEx;


public class HoodAngle {

    private final ServoEx hood;
    public DcMotorEx flywheel;

    //constants heights in inches
    public static double SHOOTER_HEIGHT = 12.0;
    public static double GOAL_HEIGHT = 39;
    private static final double HEIGHT_DIFF = GOAL_HEIGHT - SHOOTER_HEIGHT;

    //hood tuning
    private static final double SERVO_DEG_PER_HOOD = 8.125;

    public HoodAngle(HardwareMap hw,
                     double LLHeight,
                     double tagHeight) {

        hood = new ServoEx("hoodServo", 0.05);
        flywheel = hw.get(DcMotorEx.class, "shooter1");

        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


    public void aimFromDistance(double distanceInches, Telemetry telemetry) {
        if (distanceInches < 1.0) return;
        flywheel.setVelocity(3500);
        double servoPos = hoodPositionFromDistance(distanceInches, telemetry);
        telemetry.addData("hoodPos: ", servoPos);

        if (Double.isNaN(servoPos) ){
            hood.setPosition(0);
        } else {
            hood.setPosition(servoPos);
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

    public static double getProjectileAngle(double v0, double R, Telemetry telemetry) {
        double g = 386.09;

        double A = (g * R * R) / (2.0 * v0 * v0);
        double B = -R;
        double C = A + HEIGHT_DIFF;

        double D = B*B - 4*A*C;
        telemetry.addData("discrmination: ", D);

        double sqrtD = Math.sqrt(D);

        double T = (-B - sqrtD) / (2*A); //low

        // double T = (-B + sqrtD) / (2*A); //high

        return Math.atan(T); // radians
    }

    public double hoodPositionFromDistance(double distance, Telemetry telemetry) {
        double angleDeg = getProjectileAngle(getInitialVelocity(1.5), distance, telemetry);
        double servoPos = Math.abs(Range.clip((angleDeg * SERVO_DEG_PER_HOOD / 255.0), 0.0, 1.0));

            telemetry.addData("what the hood is supposed to be at: ", angleDeg);
        telemetry.addData("Servoting: ", servoPos);

        return servoPos;
    }


}