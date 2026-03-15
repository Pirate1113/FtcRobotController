package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.testing.LimelightAngle;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();

    private Turret() {}

    private ServoEx leftServo;
    private ServoEx rightServo;
    private AbsoluteAnalogEncoder encoder;

    // Limelight
    private LimelightAngle limelight;

    private double offset = 0;
    private double baseAngle = 0;

    private boolean isLoose = false;
    private boolean autoAimEnabled = false;

    public static double turretZero = 0.5;
    public static double turretServoRange = 360.0;
    public static double ENCODER_OFFSET_DEG = 187.75;

    @Override
    public void initialize() {

        leftServo = new ServoEx(RobotConstants.leftTurret);
        rightServo = new ServoEx(RobotConstants.rightTurret);

        AnalogInput ai = ActiveOpMode.hardwareMap().get(AnalogInput.class, "t_absolute");
        encoder = new AbsoluteAnalogEncoder(ai, 3.3)
                .zero(Math.toRadians(ENCODER_OFFSET_DEG));

        limelight = new LimelightAngle(
                ActiveOpMode.hardwareMap(),
                "limelight",
                13.5,
                29.5
        );

        limelight.pipelineSwitch(0);
    }

    @Override
    public void periodic() {

        // limelight
        if (autoAimEnabled && limelight.hasTarget()) {

            double yaw = limelight.getYaw();
            double currentAngle = getEncoderAngle();

            // aiming w limelight
            baseAngle = currentAngle + yaw;
        }

        if (isLoose) {
            leftServo.getServo().getController().pwmDisable();
            rightServo.getServo().getController().pwmDisable();
            return;
        }

        leftServo.getServo().getController().pwmEnable();

        double totalAngle = baseAngle + offset;

        double rawTarget = turretZero + (totalAngle / turretServoRange);

        double wrappedTarget = rawTarget % 1.0;
        if (wrappedTarget < 0) wrappedTarget += 1.0;

        leftServo.setPosition(wrappedTarget);
        rightServo.setPosition(wrappedTarget);
    }

    // control methods

    public void setBaseAngle(double angle) {
        this.baseAngle = angle;
        this.autoAimEnabled = false;
    }

    public void enableAutoAim() {
        autoAimEnabled = true;
    }

    public void disableAutoAim() {
        autoAimEnabled = false;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void addOffset(double delta) {
        this.offset += delta;
    }

    public void setLoose(boolean loose) {
        this.isLoose = loose;
    }


    public double getEncoderAngle() {
        return Math.toDegrees(encoder.getCurrentPosition());
    }

    public double getTargetAngle() {
        return baseAngle + offset;
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }
}