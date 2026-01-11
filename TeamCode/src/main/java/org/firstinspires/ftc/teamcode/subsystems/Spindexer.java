package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
import kotlin.time.Instant;

public class Spindexer implements Subsystem {
    public static final Spindexer INSTANCE = new Spindexer();
    private static final double POWER_INCREMENT = 0.2;

    private Spindexer() {
    }

    private CRServoEx servoLeft;
    private CRServoEx servoRight;
    private ServoEx ejector;
    private double powerLeft = 0.0;
    private double powerRight = 0.0;

    public void initialize() {
        servoLeft = new CRServoEx("spindexerleft");
        servoRight = new CRServoEx("spindexerright");
        ejector = new ServoEx("ejectorServo");
    }

    public Command moveLeft = new InstantCommand(() -> {
        powerLeft = 0.8;
        powerRight = -0.8;
        servoLeft.getServo().setPower(powerLeft);
        servoRight.getServo().setPower(-powerRight);
    });
    public Command moveRight = new InstantCommand(() -> {
        powerRight = 0.8;
        powerLeft = -0.8;
        servoLeft.getServo().setPower(powerLeft);
        servoRight.getServo().setPower(-powerRight);
    });
    public Command stop = new InstantCommand(() -> {
        powerLeft = 0;
        powerRight = 0;
        servoLeft.getServo().setPower(powerLeft);
        servoRight.getServo().setPower(-powerRight);
    });

    public Command eject = new InstantCommand(() -> {
            ejector.getServo().setPosition(1);
    });
    public Command uneject = new InstantCommand(() -> {
            ejector.getServo().setPosition(0.6);
    });
    public double getLeftPower() {
        return powerLeft;
    }

    public double getRightPower() {
        return powerRight;
    }
}