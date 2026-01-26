package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Auto Aim Hood Test", group="Testing")
public class AutoAimTestOpMode extends LinearOpMode {

    private HoodAngle hood;
    private LimelightAngle limelight;

    public static final double LLHeight = 14.0;
    public static final double shooterHeight = 12.0;
    public static final double tagHeight = 29.5;

    @Override
    public void runOpMode() {
        hood = new HoodAngle(hardwareMap, LLHeight, tagHeight);
        limelight = new LimelightAngle(hardwareMap, "limelight", LLHeight, tagHeight);

        telemetry.addLine("Hood and Limelight initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (limelight.hasTarget()) {
                double distance = limelight.getDistanceInches();
                hood.aimFromDistance(distance);

                double hoodServoPos = hood.hoodPositionFromDistance(distance);
                double actualRpm = hood.getFlywheelRpm();

                double velocity = hood.getInitialVelocity(0.0376565);

                telemetry.addData("Target Detected", true);
                telemetry.addData("Distance (in)", "%.1f", distance);
                telemetry.addData("Projectile Angle", "%.1f°", hood.getProjectileAngle(distance));
                telemetry.addData("Hood Servo Pos", "%.3f", hoodServoPos);

                telemetry.addData("Actual RPM", "%.0f", actualRpm);
                telemetry.addData("Velocity (in/s)", "%.0f", velocity * 39.37);
            } else {
                telemetry.addData("Target Detected", false);
            }
            telemetry.update();
        }

        hood.stop();
        limelight.stop();
    }
}