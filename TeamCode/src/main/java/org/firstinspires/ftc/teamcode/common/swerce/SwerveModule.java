package org.firstinspires.ftc.teamcode.common.swerce;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

import java.util.Locale;

public class SwerveModule {

    /* ===================== TUNING ===================== */
    public static double P = 0.6;
    public static double I = 0.0;
    public static double D = 0.1;
    public static double K_STATIC = 0.03;

    public static double MAX_SERVO = 1.0;
    public static double MAX_MOTOR = 1.0;
    public static boolean MOTOR_FLIPPING = true;

    public static double WHEEL_RADIUS = 1.4; // inches
    public static double GEAR_RATIO = 1 / (3.5 * 1.5 * 2);
    public static final double TICKS_PER_REV = 28;

    /* ===================== HARDWARE ===================== */
    private final DcMotorEx motor;
    private final CRServo servo;
    private final AbsoluteAnalogEncoder encoder;

    /* ===================== CONTROL ===================== */
    private final SimplePID rotationPID;

    /* ===================== STATE ===================== */
    private double target = 0.0;
    private double position = 0.0;
    public boolean wheelFlipped = false;
    public double lastMotorPower = 0.0;

    /* ===================== CONSTRUCTOR ===================== */
    public SwerveModule(DcMotorEx motor, CRServo servo, AbsoluteAnalogEncoder encoder) {
        this.motor = motor;
        this.servo = servo;
        this.encoder = encoder;

        MotorConfigurationType cfg = motor.getMotorType().clone();
        cfg.setAchieveableMaxRPMFraction(MAX_MOTOR);
        motor.setMotorType(cfg);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500));

        rotationPID = new SimplePID(P, I, D);
    }

    /* ===================== LOOP ===================== */
    public void read() {
        position = encoder.getCurrentPosition(); // radians 0–2π
    }

    public void update() {
        rotationPID.setPID(P, I, D);

        double current = getModuleRotation();
        double desired = target;

        double error = normalizeRadians(desired - current);

        // Shortest-path flipping
        if (MOTOR_FLIPPING && Math.abs(error) > Math.PI / 2) {
            desired = normalizeRadians(desired + Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        error = normalizeRadians(desired - current);

        double power = rotationPID.calculate(error);
        power = Range.clip(power, -MAX_SERVO, MAX_SERVO);

        // Static friction compensation
        if (Math.abs(error) > 0.02) {
            power += K_STATIC * Math.signum(error);
        }

        if (Double.isNaN(power)) power = 0;
        servo.setPower(power);
    }

    /* ===================== CONTROL ===================== */
    public void setTargetRotation(double radians) {
        target = normalizeRadians(radians);
    }

    public void setMotorPower(double power) {
        if (wheelFlipped) power *= -1;
        lastMotorPower = power;
        motor.setPower(power);
    }

    /* ===================== GETTERS ===================== */
    public double getModuleRotation() {
        // Adjust this offset ONLY if your encoder mounting requires it
        return normalizeRadians(position - Math.PI);
    }

    public double getTargetRotation() {
        return target;
    }

    public double getWheelPosition() {
        return encoderTicksToInches(motor.getCurrentPosition());
    }

    public double getWheelVelocity() {
        return encoderTicksToInches(motor.getVelocity());
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    public String getTelemetry(String name) {
        return String.format(
                Locale.ENGLISH,
                "%s | rot=%.2f tgt=%.2f flip=%b pwr=%.2f",
                name, getModuleRotation(), target, wheelFlipped, lastMotorPower
        );
    }

    /* ===================== UTIL ===================== */
    private double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    /* ===================== SIMPLE PID ===================== */
    private static class SimplePID {
        private double kP, kI, kD;
        private double integral = 0.0;
        private double lastError = 0.0;

        SimplePID(double p, double i, double d) {
            kP = p;
            kI = i;
            kD = d;
        }

        void setPID(double p, double i, double d) {
            kP = p;
            kI = i;
            kD = d;
        }

        double calculate(double error) {
            integral += error;
            double derivative = error - lastError;
            lastError = error;
            return kP * error + kI * integral + kD * derivative;
        }

        void reset() {
            integral = 0.0;
            lastError = 0.0;
        }
    }
}