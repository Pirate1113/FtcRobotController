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
            .posPid(0.1, 0.0, 0.01)
            .build();

    // Position tracking for left servo
    private double totalAngleLeft = 0.0;
    private double previousAngleLeft = 0.0;
    private double velocityLeft = 0.0;

    private double startLeftPos;
    // Position tracking for right servo
    private double totalAngleRight = 0.0;
    private double previousAngleRight = 0.0;

    private double powerLeft;

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

        startLeftPos = servoLeft.getCurrentPosition();
    }

    @Override
    public void periodic() {
        // Update position tracking for both servos
        updateLeftPosition();

        // Get actual velocity from servo, combine with unwrapped position
        velocityLeft = servoLeft.getVelocity();
        powerLeft = controllerLeft.calculate(servoLeft.getState());

        servoLeft.setPower(powerLeft);
        servoRight.setPower(powerLeft);
    }

    // ===== POSITION TRACKING =====
    void updateLeftPosition() {

        double currentAngle = servoLeft.getCurrentPosition() - startLeftPos;
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

    public Command b1 = new RunToPosition(
            controllerLeft,
            0,
            0.1   // absolute tolerance in units
    );
//            InstantCommand(() -> {
//        controllerLeft.setGoal(new KineticState(0,0));
////        controllerRight.setGoal(new KineticState(0, 0.0));
//    }).named("Stop");
    public Command b2 = new InstantCommand(() -> {
        controllerLeft.setGoal(new KineticState(Math.PI/1.5, 0.0));
//        controllerRight.setGoal(new KineticState(Math.PI/1.5, 0.0));
    });
    public Command b3 = new InstantCommand(() -> {
        controllerLeft.setGoal(new KineticState(2*Math.PI/1.5, 0.0));
//        controllerRight.setGoal(new KineticState(2*Math.PI/1.5, 0.0));
    });
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

    public String getLeftGoal() {
        return controllerLeft.getGoal().toString();
    }
    public double getLeftPower() {
        return powerLeft;
    }
//    public String getRightGoal() {
//        return controllerRight.getGoal().toString();
//    }
}