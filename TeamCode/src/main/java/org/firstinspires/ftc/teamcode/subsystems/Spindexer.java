package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Spindexer implements Subsystem {
    public static final Spindexer INSTANCE = new Spindexer();
    private static final double POWER_INCREMENT = 0.2;

    private Spindexer() {}

    private CRServoEx servoLeft;
    private CRServoEx servoRight;

    private double powerLeft = 0.0;
    private double powerRight = 0.0;

    public void initialize() {
        servoLeft = new CRServoEx("spindexerleft");
        servoRight = new CRServoEx("spindexerright");
    }

    // ===== SYNCHRONIZED CONTROL =====

    /** Run both servos at full power to intake samples */
    public Command intake() {
        powerLeft = 1.0;
        powerRight = 1.0;
        return new SetPower(servoLeft, 1.0)
            .and(new SetPower(servoRight, 1.0))
            .named("Intake");
    }

    /** Run both servos. in reverse to eject samples */
    public Command outtake() {
        powerLeft = -1.0;
        powerRight = -1.0;
        return new SetPower(servoLeft, -1.0)
            .and(new SetPower(servoRight, -1.0))
            .named("Outtake");
    }

    /** Stop both servos */
    public Command stop() {
        powerLeft = 0.0;
        powerRight = 0.0;
        return new SetPower(servoLeft, 0.0)
            .and(new SetPower(servoRight, 0.0))
            .named("Stop Spindexer");
    }

    /** Set both servos to a specific power */
    public Command setPower(double power) {
        powerLeft = power;
        powerRight = power;
        return new SetPower(servoLeft, power)
            .and(new SetPower(servoRight, power))
            .named("Set Spindexer Power");
    }

    // ===== INDEPENDENT SERVO CONTROL =====

    /** Increase left servo power by increment */
    public Command increaseLeft() {
        powerLeft = Math.min(1.0, powerLeft + POWER_INCREMENT);
        return new SetPower(servoLeft, powerLeft).named("Increase Left Power");
    }

    /** Decrease left servo power by increment */
    public Command decreaseLeft() {
        powerLeft = Math.max(-1.0, powerLeft - POWER_INCREMENT);
        return new SetPower(servoLeft, powerLeft).named("Decrease Left Power");
    }

    /** Increase right servo power by increment */
    public Command increaseRight() {
        powerRight = Math.min(1.0, powerRight + POWER_INCREMENT);
        return new SetPower(servoRight, powerRight).named("Increase Right Power");
    }

    /** Decrease right servo power by increment */
    public Command decreaseRight() {
        powerRight = Math.max(-1.0, powerRight - POWER_INCREMENT);
        return new SetPower(servoRight, powerRight).named("Decrease Right Power");
    }

    /** Set left servo to specific power */
    public Command setLeftPower(double power) {
        powerLeft = Math.max(-1.0, Math.min(1.0, power));
        return new SetPower(servoLeft, powerLeft).named("Set Left Power");
    }

    /** Set right servo to specific power */
    public Command setRightPower(double power) {
        powerRight = Math.max(-1.0, Math.min(1.0, power));
        return new SetPower(servoRight, powerRight).named("Set Right Power");
    }

    // ===== GETTERS FOR TELEMETRY =====

    public double getLeftPower() {
        return powerLeft;
    }

    public double getRightPower() {
        return powerRight;
    }
}