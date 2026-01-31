package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
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
    private final PIDCoefficients pidValues = new PIDCoefficients(0.025, 0.0025, 0.0);
    private final ControlSystem controllerLeft = ControlSystem.builder()
            .angular(AngleType.RADIANS, feedback -> feedback.posPid(pidValues))
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
            0.00,
            () -> { return ActiveOpMode.hardwareMap().get(AnalogInput.class, "analogLeft"); },
            () -> { return ActiveOpMode.hardwareMap().get(CRServo.class, "spindexerleft"); });
        servoRight = new FeedbackCRServoEx(
                0.00,
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

        // Create KineticState with unwrapped position and velocity (like SwerveModule)
        KineticState currentState = new KineticState(totalAngleLeft, velocityLeft);
        powerLeft = controllerLeft.calculate(currentState);

        servoLeft.setPower(powerLeft);
        servoRight.setPower(powerLeft);
    }
    public void updateLeftPosition(){
        totalAngleLeft = Angle.Companion.wrapAnglePiToPi(servoLeft.getCurrentPosition());
    } // this comes out [-pi, pi)

    public Command b1 = new RunToPosition(
            controllerLeft,
            0, // -offset
            0.05   // absolute tolerance in units
    );
    public Command b2 = new RunToPosition(
            controllerLeft,
            Math.PI/1.5, // - offset
            0.05   // absolute tolerance in units
    );
    public Command b3 = new RunToPosition(
            controllerLeft,
            -Math.PI/1.5, // - offset
            0.05   // absolute tolerance in units
    );
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
}