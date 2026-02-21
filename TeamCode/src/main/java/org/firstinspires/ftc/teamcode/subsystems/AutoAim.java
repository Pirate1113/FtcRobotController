package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.testing.LimelightAngle;

public class AutoAim implements Subsystem {
    public static final AutoAim INSTANCE = new AutoAim();

    private AutoAim() {}

    private static final double LL_HEIGHT = 14;
    private static final double TAG_HEIGHT = 29.5;

    // Hardware
    private LimelightAngle limelight;
    private ServoEx hood;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    // State
    private boolean enabled = false;
    private double distance = 0;
    private double hoodPosition = 0;
    private double targetVelocity = 0;

    @Override
    public void initialize() {
        limelight = new LimelightAngle(
            ActiveOpMode.hardwareMap(),
            "limelight",
            LL_HEIGHT,
            TAG_HEIGHT
        );
        limelight.pipelineSwitch(0);

        hood = new ServoEx("hoodServo");

        shooter1 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "shooter1");
        shooter2 = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "shooter2");
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void periodic() {
        if (!enabled) return;

        distance = limelight.getDistanceInches();

        // Interpolated hood position
        hoodPosition = -36.79717 * Math.pow(distance, -1.08794) + 0.494168;
        hoodPosition = Range.clip(hoodPosition, 0.0, 1.0);
        hood.setPosition(hoodPosition);

        // Interpolated flywheel velocity (in ticks/sec)
        targetVelocity = (10.88255 * distance + 2691.0285) * 28 / 60;
        targetVelocity = Range.clip(targetVelocity, 0, 6000);
        shooter1.setVelocity(-targetVelocity);
        shooter2.setVelocity(-targetVelocity);
    }

    // Commands
    public final Command enable = new InstantCommand(() -> enabled = true);
    public final Command on = new InstantCommand(() -> {
        enabled = false;
        shooter1.setPower(-1);
        shooter2.setPower(-1);
    });

    public final Command off = new InstantCommand(() -> {
        enabled = false;
        shooter1.setPower(0);
        shooter2.setPower(0);
    });

    public final Command reverse = new InstantCommand(() -> {
        enabled = false;
        shooter1.setPower(-0.2);
        shooter2.setPower(-0.2);
    });

    // Telemetry getters
    public double getDistance() { return distance; }
    public double getHoodPosition() { return hoodPosition; }
    public double getTargetRPM() { return targetVelocity * 60 / 28; }
    public double getCurrentRPM() { return Math.abs(shooter1.getVelocity()) * 60 / 28; }
    public boolean isEnabled() { return enabled; }
    public boolean hasTarget() { return limelight != null && limelight.hasTarget(); }
}