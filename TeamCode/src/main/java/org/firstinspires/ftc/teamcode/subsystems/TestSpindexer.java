package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;
import dev.nextftc.hardware.controllable.RunToPosition;

public class TestSpindexer implements Subsystem {
    public static final TestSpindexer INSTANCE = new TestSpindexer();

    private TestSpindexer() {}

    // Hardware
    private FeedbackCRServoEx servoLeft;
    private FeedbackCRServoEx servoRight;

    // Control Systems (one for each servo)
    private final ControlSystem controllerLeft = ControlSystem.builder()
            .build();

    // Position tracking for left servo
    private double totalAngleLeft = 0.0;
    private double previousAngleLeft = 0.0;

    // Position tracking for right servo
    private double totalAngleRight = 0.0;
    private double previousAngleRight = 0.0;
    @Override
    public void initialize() {
        servoLeft = new FeedbackCRServoEx(
            0.01,
            () -> { return ActiveOpMode.hardwareMap().get(AnalogInput.class, "analogLeft"); },
            () -> { return ActiveOpMode.hardwareMap().get(CRServo.class, "spindexerleft"); });
        servoRight = new FeedbackCRServoEx(
                0.01,
                () -> { return ActiveOpMode.hardwareMap().get(AnalogInput.class, "analogRight"); },
                () -> { return ActiveOpMode.hardwareMap().get(CRServo.class, "spindexerright"); });
    }

    @Override
    public void periodic() {
        // Update position tracking for both servos
        updateLeftPosition();

        // Calculate and apply control outputs


        servoLeft.setPower(powerLeft);
    }

    // ===== POSITION TRACKING =====
    void updateLeftPosition() {
        double deltaAngle = currentAngle - previousAngleLeft;

        // Handle wrapping at 2π
        if (deltaAngle > Math.PI) {
            deltaAngle -= 2 * Math.PI;
        } else if (deltaAngle < -Math.PI) {
            deltaAngle += 2 * Math.PI;
        }

        totalAngleLeft += deltaAngle;
        previousAngleLeft = currentAngle;
    }

    private void updateRightPosition() {
        double currentAngle = servoRight.getCurrentPosition();
        double deltaAngle = currentAngle - previousAngleRight;

        // Handle wrapping at 2π
        if (deltaAngle > Math.PI) {
            deltaAngle -= 2 * Math.PI;
        } else if (deltaAngle < -Math.PI) {
            deltaAngle += 2 * Math.PI;
        }

        totalAngleRight += deltaAngle;
        previousAngleRight = currentAngle;

    }

    }).named("Stop");

    public double getLeftPosition() {
        return totalAngleLeft;
    }

    public double getRightPosition() {
        return totalAngleRight;
    }

    public double getLeftRawPosition() {
        return servoLeft.getCurrentPosition();
    }

    public double getRightRawPosition() {
        return servoRight.getCurrentPosition();
    }
}