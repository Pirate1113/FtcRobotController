package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Spindexer implements Subsystem {

    public static final Spindexer INSTANCE = new Spindexer();

    private Spindexer() {
    }

    // consrants
    private static final double NOMINAL_VOLTAGE = 11.4;

    // ===== Hardware =====
    private FeedbackCRServoEx servoLeft;
    private FeedbackCRServoEx servoRight;
    private ServoEx ejector;
    private VoltageSensor voltageSensor;

    private static double ejectorPos = 0;

    // control
    private final PIDCoefficients pid =
            new PIDCoefficients(0.56, 0.0, 0.03);

    private final ControlSystem controller =
            ControlSystem.builder()
                    .angular(AngleType.RADIANS, fb -> fb.posPid(pid))
                    .build();

    private double totalAngle = 0.0;
    private double previousAngle = 0.0;
    private double power = 0.0;

    @Override
    public void initialize() {
        voltageSensor = ActiveOpMode.hardwareMap().voltageSensor.iterator().next();

        servoLeft = new FeedbackCRServoEx(
                0.0,
                () -> ActiveOpMode.hardwareMap().get(AnalogInput.class, "analogLeft"),
                () -> ActiveOpMode.hardwareMap().get(CRServo.class, "spindexerleft")
        );

        servoRight = new FeedbackCRServoEx(
                0.0,
                () -> ActiveOpMode.hardwareMap().get(AnalogInput.class, "analogRight"),
                () -> ActiveOpMode.hardwareMap().get(CRServo.class, "spindexerright")
        );

        ejector = new ServoEx("finger");

        previousAngle = servoLeft.getCurrentPosition();
        totalAngle = 0.0;
    }

    // ===== Angle Unwrapping =====
    private void updatePosition() {
        double current = servoLeft.getCurrentPosition();
        double delta = current - previousAngle;

        if (delta > Math.PI) delta -= 2 * Math.PI;
        if (delta < -Math.PI) delta += 2 * Math.PI;

        totalAngle += delta;
        previousAngle = current;
    }

    @Override
    public void periodic() {
        updatePosition();

        KineticState state = new KineticState(
                totalAngle,
                servoLeft.getVelocity()
        );

        double raw = controller.calculate(state);

        double voltage = Math.max(voltageSensor.getVoltage(), 9.5);
        double compensated = raw * (NOMINAL_VOLTAGE / voltage);
        compensated = Math.max(-1.0, Math.min(1.0, compensated));

        servoLeft.setPower(-compensated);
        servoRight.setPower(-compensated);

        power = compensated;
    }

    private void setGoal(double angleRad) {
        controller.setGoal(new KineticState(angleRad, 0.0));
    }

    // commands
    public final Command b1 = new InstantCommand(() -> setGoal(-2.3419));
    public final Command b2 = new InstantCommand(() -> setGoal(-2.3419 + Math.PI / 1.5));
    public final Command b3 = new InstantCommand(() -> setGoal(-2.3419 + 2 * Math.PI / 1.5));

    public final Command i1 = new InstantCommand(() -> setGoal(-1.2376));
    public final Command i2 = new InstantCommand(() -> setGoal(-1.2376 + Math.PI / 1.5));
    public final Command i3 = new InstantCommand(() -> setGoal(-1.2376 + 2 * Math.PI / 1.5));

    public final Command shoot =
            new SequentialGroup(b1, b2, b3);

    public final Command eject =
            new InstantCommand(() -> {
                ejectorPos = 1;
                ejector.getServo().setPosition(ejectorPos);
            });

    public final Command uneject =
            new InstantCommand(() -> {
                ejectorPos = 0;
                ejector.getServo().setPosition(ejectorPos);
            });

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