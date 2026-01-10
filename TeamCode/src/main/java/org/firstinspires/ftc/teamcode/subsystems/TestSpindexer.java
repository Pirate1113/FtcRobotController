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
            .posPid(1.0, 0.0, 0.1)  // Start with these PID values, tune as needed
            .build();

    private final ControlSystem controllerRight = ControlSystem.builder()
            .posPid(1.0, 0.0, 0.1)
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
        updateRightPosition();

        // Calculate and apply control outputs
        double powerLeft = controllerLeft.calculate(servoLeft.getState());

        double powerRight = controllerRight.calculate(servoRight.getState());

        servoLeft.setPower(powerLeft);
        servoRight.setPower(powerRight);
    }

    // ===== POSITION TRACKING =====
    void updateLeftPosition() {
        double currentAngle = servoLeft.getCurrentPosition();
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

    // ===== POSITION-BASED COMMANDS =====

    /** Move left by rotating servos in opposite directions */
    public Command moveLeft = new RunToPosition(controllerLeft, totalAngleLeft + Math.PI)
                .and(new RunToPosition(controllerRight, totalAngleRight - Math.PI))
                .named("Move Left");

    /** Move right by rotating servos in opposite directions */
    public Command moveRight = new RunToPosition(controllerLeft, totalAngleLeft - Math.PI)
                .and(new RunToPosition(controllerRight, totalAngleRight + Math.PI))
                .named("Move Right");

    /** Rotate both servos to specific positions */
    public Command goToPosition(double leftTarget, double rightTarget) {
        return new RunToPosition(controllerLeft, leftTarget)
                .and(new RunToPosition(controllerRight, rightTarget))
                .named("Go To Position");
    }

    /** Stop at current positions */
    public Command stop() {
        return new InstantCommand(() -> {
            controllerLeft.setGoal(new KineticState(totalAngleLeft, 0.0));
            controllerRight.setGoal(new KineticState(totalAngleRight, 0.0));
        }).named("Stop");
    }

    /** Index forward (rotate one slot) - adjust angle as needed for your mechanism */
    public Command indexForward() {
        double indexAngle = Math.PI / 1.5; // 60 degrees, adjust for your spindexer
        return new RunToPosition(controllerLeft, totalAngleLeft + indexAngle)
                .and(new RunToPosition(controllerRight, totalAngleRight + indexAngle))
                .named("Index Forward");
    }

    /** Index backward (rotate one slot back) */
    public Command indexBackward() {
        double indexAngle = Math.PI / 1.5; // 60 degrees, adjust for your spindexer
        return new RunToPosition(controllerLeft, totalAngleLeft - indexAngle)
                .and(new RunToPosition(controllerRight, totalAngleRight - indexAngle))
                .named("Index Backward");
    }

    // ===== MANUAL POWER CONTROL (if you need it) =====

    /** Manually set power (bypasses PID control) */
    public Command setManualPower(double leftPower, double rightPower) {
        return new InstantCommand(() -> {
            servoLeft.setPower(leftPower);
            servoRight.setPower(rightPower);
        }).named("Manual Power");
    }

    // ===== GETTERS FOR TELEMETRY =====

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