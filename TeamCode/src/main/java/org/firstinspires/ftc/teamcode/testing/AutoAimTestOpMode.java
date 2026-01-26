package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Auto Aim Hood Test", group="Testing")
public class AutoAimTestOpMode extends LinearOpMode {

    private HoodAngle hood;
    private LimelightAngle limelight;

    // Heights in inches
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

                //telemetry

                double hoodServoPos = hood.hoodPositionFromDistance(distance);
                double actualRpm = hood.getFlywheelRpm();
                double targetRpm = HoodAngle.BASE_RPM + distance * HoodAngle.RPM_PER_INCH;
                double velocity = hood.getInitialVelocity(0.0376565);

                telemetry.addData("Target Detected", true);
                telemetry.addData("Distance (in)", "%.1f", distance);
                telemetry.addData("Hood Servo Pos", "%.3f", hoodServoPos);
                telemetry.addData("Target FAKE RPM", "%.0f", targetRpm);
                telemetry.addData("Actual REAL RPM", "%.0f", actualRpm);
                telemetry.addData("Velocity (m/s)", "%.1f", velocity);

            } else {
                telemetry.addData("Target Detected", false);
            }
            telemetry.update();
        }

        hood.stop();
        limelight.stop();
    }
}