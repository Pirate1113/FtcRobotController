package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.VoltageSensor;


import java.time.Duration;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.ServoEx;

public class Spindexer implements Subsystem {
    public static final Spindexer INSTANCE = new Spindexer();

    private Spindexer() {}
    private VoltageSensor voltageSensor;
    private static final double NOMINAL_VOLTAGE = 11.4;


    // Hardware
    private FeedbackCRServoEx servoLeft;
    private FeedbackCRServoEx servoRight;
    private ServoEx ejector;
    private static double ejectorPos = 0;

    private final PIDCoefficients pidValues = new PIDCoefficients(0.56, 0.0000000009, 0.03);
    private ControlSystem controllerLeft = ControlSystem.builder()
            .angular(AngleType.RADIANS, feedback -> feedback.posPid(pidValues))
            .build();

    // Position tracking for left servo
    private double totalAngleLeft = 0.0;
    private double previousAngleLeft = 0.0;
    private double velocityLeft = 0.0;
    private double leftOffset;
    private double powerLeft;
    private static double previousAngle;

    @Override
    public void initialize() {
        voltageSensor = ActiveOpMode.hardwareMap().voltageSensor.iterator().next();

        servoLeft = new FeedbackCRServoEx(
                0.00,
                () -> ActiveOpMode.hardwareMap().get(AnalogInput.class, "analogLeft"),
                () -> ActiveOpMode.hardwareMap().get(CRServo.class, "spindexerleft")
        );

        servoRight = new FeedbackCRServoEx(
                0.00,
                () -> ActiveOpMode.hardwareMap().get(AnalogInput.class, "analogRight"),
                () -> ActiveOpMode.hardwareMap().get(CRServo.class, "spindexerright")
        );

        ejector = new ServoEx("finger");

        previousAngleLeft = servoLeft.getCurrentPosition();
        totalAngleLeft = 0.0;
    }

    private void updateLeftPosition() {
        double current = servoLeft.getCurrentPosition();
        double delta = current - previousAngleLeft;

        if (delta > Math.PI) delta -= 2 * Math.PI;
        if (delta < -Math.PI) delta += 2 * Math.PI;

        totalAngleLeft += delta;
        previousAngleLeft = current;
    }

    @Override
    public void periodic() {
        updateLeftPosition();

        velocityLeft = servoLeft.getVelocity();
        KineticState currentState = new KineticState(totalAngleLeft, velocityLeft);

        if (ejectorPos == 0) {
            double rawPower = controllerLeft.calculate(currentState);

            double voltage = voltageSensor.getVoltage();
            double compensatedPower = rawPower * (NOMINAL_VOLTAGE / voltage);

            compensatedPower = Math.max(-1.0, Math.min(1.0, compensatedPower));

            servoLeft.setPower(-compensatedPower);
            servoRight.setPower(-compensatedPower);

            powerLeft = compensatedPower;
        }
    }

    public Command b1 = new RunToPosition(
            controllerLeft,
            -2.3419, // -offset
            0.05   // absolute tolerance in units
    );
    public Command b2 = new RunToPosition(
            controllerLeft,
            -2.3419+Math.PI/1.5, // - offset
            0.05   // absolute tolerance in units
    );
    public Command b3 = new RunToPosition(
            controllerLeft,
            -2.3419+2*Math.PI/1.5, // - offset
            0.05   // absolute tolerance in units
    );
    public Command i1 = new RunToPosition(
            controllerLeft,
            -1.2376, // -offset
            0.05   // absolute tolerance in units
    );
    public Command i2 = new RunToPosition(
            controllerLeft,
            -1.2376+Math.PI/1.5, // - offset
            0.05   // absolute tolerance in units
    );
    public Command i3 = new RunToPosition(
            controllerLeft,
            -1.2376+2*Math.PI/1.5, // - offset
            0.05   // absolute tolerance in units
    );
    public Command shoot = new SequentialGroup(
            b1,
            new Delay(1),
            b2,
            new Delay(1),
            b3
    );
    public Command uneject = new InstantCommand(() -> {
        ejectorPos = 0;
        ejector.getServo().setPosition(ejectorPos);
    });
    public Command eject = new InstantCommand(() -> {
        ejectorPos = 1;
        ejector.getServo().setPosition(ejectorPos);
    });
    public double getEjectorPos() {
        return ejector.getServo().getPosition();
    }
    public double getLeftPosition() {
        return totalAngleLeft;
    }

    public double getLeftRawPosition() {
        return servoLeft.getCurrentPosition();
    }
    public String getLeftGoal() {
        return controllerLeft.getGoal().toString();
    }
    public double getLeftPower() {
        return powerLeft;
    }
}

//TUNNING CODE:
//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import dev.nextftc.control.ControlSystem;
//import dev.nextftc.control.KineticState;
//import dev.nextftc.control.feedback.AngleType;
//import dev.nextftc.control.feedback.PIDCoefficients;
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.utility.InstantCommand;
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.core.units.Angle;
//import dev.nextftc.ftc.ActiveOpMode;
//import dev.nextftc.hardware.impl.FeedbackCRServoEx;
//import dev.nextftc.hardware.controllable.RunToPosition;
//
//@Config
//public class TestSpindexer implements Subsystem {
//    public static final TestSpindexer INSTANCE = new TestSpindexer();
//
//    private TestSpindexer() {}
//
//    // Hardware
//    private FeedbackCRServoEx servoLeft;
//    private FeedbackCRServoEx servoRight;
//    public static double kP = 0.025;
//    public static double kI = 0;
//    public static double kD = 0.0;
//
//    // Control Systems (one for each servo)
//    private final PIDCoefficients pidValues = new PIDCoefficients(0.025, 0.0025, 0.0);
//    private ControlSystem controllerLeft = ControlSystem.builder()
//            .angular(AngleType.RADIANS, feedback -> feedback.posPid(pidValues))
//            .build();
//
//    // Position tracking for left servo
//    private double totalAngleLeft = 0.0;
//    private double previousAngleLeft = 0.0;
//    private double velocityLeft = 0.0;
//    Telemetry dashboardTelemetry;
//    private double leftOffset;
//
//    FtcDashboard dashboard;
//    private double powerLeft;
//    public static double goal = 0;
//    private static double previousAngle;
//
//    @Override
//    public void initialize() {
//        servoLeft = new FeedbackCRServoEx(
//                0.00,
//                () -> { return ActiveOpMode.hardwareMap().get(AnalogInput.class, "analogLeft"); },
//                () -> { return ActiveOpMode.hardwareMap().get(CRServo.class, "spindexerleft"); });
//        servoRight = new FeedbackCRServoEx(
//                0.00,
//                () -> { return ActiveOpMode.hardwareMap().get(AnalogInput.class, "analogRight"); },
//                () -> { return ActiveOpMode.hardwareMap().get(CRServo.class, "spindexerright"); });
//
//        controllerLeft = ControlSystem.builder()
//                .angular(AngleType.RADIANS, feedback -> feedback.posPid(new PIDCoefficients(kP, kI, kD)))
//                .build();
//        controllerLeft.setGoal(new KineticState(goal));
//        dashboard = FtcDashboard.getInstance();
//
//        dashboardTelemetry = dashboard.getTelemetry();
//
//    }
//
//    @Override
//    public void periodic() {
//        // Update position tracking for both servos
//        updateLeftPosition();
//        // Rebuild controller to pick up dashboard-tuned PID values, then set goal
//        // Get actual velocity from servo, combine with unwrapped position
//        velocityLeft = servoLeft.getVelocity();
//
//        // Create KineticState with unwrapped position and velocity (like SwerveModule)
//        KineticState currentState = new KineticState(totalAngleLeft, velocityLeft);
//        powerLeft = controllerLeft.calculate(currentState);
//
//        servoLeft.setPower(-powerLeft);
//        servoRight.setPower(-powerLeft);
//
//        dashboardTelemetry.addData("Power", powerLeft);
//        dashboardTelemetry.addData("Goal", controllerLeft.getGoal().getPosition());
//        dashboardTelemetry.addData("Pos", totalAngleLeft);
//        dashboardTelemetry.addData("Error", totalAngleLeft-controllerLeft.getGoal().getPosition());
//        dashboardTelemetry.update();
//    }
//    public void updateLeftPosition(){
//        totalAngleLeft = Angle.Companion.wrapAnglePiToPi(servoLeft.getCurrentPosition());
//    } // this comes out [-pi, pi)
//
//    public Command b1 = new RunToPosition(
//            controllerLeft,
//            goal, // -offset
//            0.05   // absolute tolerance in units
//    );
//    public Command b2 = new RunToPosition(
//            controllerLeft,
//            goal, // - offset
//            0.05   // absolute tolerance in units
//    );
//    public Command b3 = new RunToPosition(
//            controllerLeft,
//            goal, // - offset
//            0.05   // absolute tolerance in units
//    );
//    public double getLeftPosition() {
//        return totalAngleLeft;
//    }
//
//    public double getLeftRawPosition() {
//        return servoLeft.getCurrentPosition();
//    }
//
//    public double getRightRawPosition() {
//        return servoRight.getCurrentPosition();
//    }
//
//    public String getLeftGoal() {
//        return controllerLeft.getGoal().toString();
//    }
//    public double getLeftPower() {
//        return powerLeft;
//    }
//}