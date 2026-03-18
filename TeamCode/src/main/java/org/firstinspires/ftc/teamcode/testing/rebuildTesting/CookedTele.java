package org.firstinspires.ftc.teamcode.testing.rebuildTesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.swerce.SwerveDrivetrain;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Cooked Tele")
public class CookedTele extends NextFTCOpMode {

    private Telemetry dashboardTelemetry;
    
    // Hardware members
    private DcMotorEx leftShooter, rightShooter;
    private DcMotorEx frontIntake;
    private Servo hoodServo, turretLeft, turretRight, gateServo;
    private CRServo feedServo;

    // Control State
    private double shooterPower = 0.8;
    private double hoodPosition = 0.5;
    private final double HOOD_INCREMENT = 0.005;
    private final double POWER_INCREMENT = 0.05;
    private boolean intakeOn = false;

    public CookedTele() {
        addComponents(
                new SubsystemComponent(SwerveDrivetrain.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Initialize Shooter
        leftShooter = hardwareMap.get(DcMotorEx.class, RobotConstants.leftShooter);
        rightShooter = hardwareMap.get(DcMotorEx.class, RobotConstants.rightShooter);
        leftShooter.setDirection(DcMotorEx.Direction.REVERSE);

        // Initialize Intake
        frontIntake = hardwareMap.get(DcMotorEx.class, RobotConstants.leftIntake);

        // Initialize Servos
        hoodServo = hardwareMap.get(Servo.class, RobotConstants.hood_name);
        turretLeft = hardwareMap.get(Servo.class, RobotConstants.leftTurret);
        turretRight = hardwareMap.get(Servo.class, RobotConstants.rightTurret);
        gateServo = hardwareMap.get(Servo.class, RobotConstants.gate);

        feedServo = hardwareMap.get(CRServo.class, "feed_servo");
        
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void onStartButtonPressed() {
        // Toggle intake with 'A'
        Gamepads.gamepad1().a().whenBecomesTrue(() -> intakeOn = !intakeOn);

        // Discrete increments for shooter power
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> 
                shooterPower = Math.min(1.0, shooterPower + POWER_INCREMENT));
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> 
                shooterPower = Math.max(0.0, shooterPower - POWER_INCREMENT));

        // Using whenTrue for continuous adjustment of hood position
        Gamepads.gamepad1().dpadUp().whenTrue(() -> 
                hoodPosition = Math.min(1.0, hoodPosition + HOOD_INCREMENT));
        Gamepads.gamepad1().dpadDown().whenTrue(() -> 
                hoodPosition = Math.max(0.0, hoodPosition - HOOD_INCREMENT));
    }

    @Override
    public void onUpdate() {

        BindingManager.update();

        // ── Flywheel (Shooter) ──────────────────────────────────────────────
        // Right trigger to fire
        double rtValue = Gamepads.gamepad1().rightTrigger().get();
        if (rtValue > 0.1) {
            leftShooter.setPower(shooterPower);
            rightShooter.setPower(shooterPower);
        } else {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
        }

        // ── Hood Position Apply ─────────────────────────────────────────────
        hoodServo.setPosition(hoodPosition);

        // ── Intake ──────────────────────────────────────────────────────────
        frontIntake.setPower(intakeOn ? 1.0 : 0.0);
        feedServo.setPower(intakeOn ? 1.0 : 0.0);

        // ── Static Hardware ─────────────────────────────────────────────────
        turretLeft.setPosition(0.0);
        turretRight.setPosition(0.0);
        gateServo.setPosition(0.75);

        // ── Telemetry ───────────────────────────────────────────────────────
        dashboardTelemetry.addData("Intake (A to Toggle)", intakeOn ? "ON" : "OFF");
        dashboardTelemetry.addData("Target Shooter Power", "%.2f", shooterPower);
        dashboardTelemetry.addData("Hood Position", "%.3f", hoodPosition);
        dashboardTelemetry.update();
    }
}
