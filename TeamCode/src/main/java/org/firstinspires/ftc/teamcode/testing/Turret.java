package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;

public class Turret implements Subsystem {

    public static final Turret INSTANCE = new Turret();
    private Turret () {}

    private Servo leftServo;
    private Servo rightServo;

    private double currentAngle = 90.0; // degrees
    private final double servoMin = 0.0;
    private final double servoMax = 1.0;
    private final double servoRangeDegrees = 180.0;

    @Override
    public void initialize() {
        HardwareMap hwMap = dev.nextftc.ftc.ActiveOpMode.hardwareMap();
        leftServo = hwMap.get(Servo.class, "turretLeft");
        rightServo = hwMap.get(Servo.class, "turretRight");

        setAngle(currentAngle);
    }

    public void setAngle(double angle) {
        angle = Math.max(0, Math.min(servoRangeDegrees, angle));
        currentAngle = angle;

        double pos = servoMin + (angle / servoRangeDegrees) * (servoMax - servoMin);
        leftServo.setPosition(pos);
        rightServo.setPosition(1.0 - pos);
    }

    public void adjustAngle(double delta) {
        setAngle(currentAngle + delta);
    }

    public double getAngle() {
        return currentAngle;
    }

    @Override
    public void periodic() {

    }
}