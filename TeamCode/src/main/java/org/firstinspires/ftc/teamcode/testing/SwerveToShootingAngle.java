package org.firstinspires.ftc.teamcode.testing;

import dev.nextftc.core.subsystems.Subsystem;

public class SwerveToShootingAngle implements Subsystem {

    private final SwerveDrivetrain drivetrain;
    private double maxTurnPower = 0.5;  // limit to avoid spinning too fast
    private double kP = 0.02;           // tUnE

    public SwerveToShootingAngle() {
        this.drivetrain = SwerveDrivetrain.INSTANCE;
    }

    @Override
    public void initialize() {
        drivetrain.initialize();
    }

    // Call periodically to turn robot toward a target yaw (degrees)
    public void turnToTarget(double yawDeg) {
        double turnPower = yawDeg * kP;

        // limit turn power
        turnPower = Math.max(-maxTurnPower, Math.min(maxTurnPower, turnPower));

        drivetrain.frontLeftModule.setMotorPower(turnPower);
        drivetrain.backLeftModule.setMotorPower(turnPower);
        drivetrain.frontRightModule.setMotorPower(-turnPower);
        drivetrain.backRightModule.setMotorPower(-turnPower);
    }

    public void stop() {
        drivetrain.frontLeftModule.setMotorPower(0);
        drivetrain.backLeftModule.setMotorPower(0);
        drivetrain.frontRightModule.setMotorPower(0);
        drivetrain.backRightModule.setMotorPower(0);
    }

    @Override
    public void periodic() {
    }
}