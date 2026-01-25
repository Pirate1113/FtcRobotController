package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.testing.HoodAngle.SERVO_INTERCEPT;
import static org.firstinspires.ftc.teamcode.testing.HoodAngle.SERVO_SLOPE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Auto Aim Hood Test", group="Testing")
public class AutoAimTestOpMode extends LinearOpMode {

    private HoodAngle hood;
    private LimelightAngle limelight;

    // needs tuning of course
    public static final double shooterHeight = 12.0;
    public static final double tagHeight = 37.0;

    @Override
    public void runOpMode() {

        // hoodangle
        hood = new HoodAngle(hardwareMap, shooterHeight, tagHeight);

        // yes LIMELIGHT yes yez
        limelight = new LimelightAngle(hardwareMap, "limelight", shooterHeight, tagHeight);

        telemetry.addLine("Hood and Limelight initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (limelight.hasTarget()) {
                // get distance
                double distance = limelight.getDistanceInches();

                // autoaim
                hood.aimFromDistance(distance);

                double verticalDiff = tagHeight - shooterHeight;
                double angleRad =
                        Math.atan((verticalDiff + Math.sqrt(
                                verticalDiff * verticalDiff + distance * distance))
                                / distance);
                double angleDeg = Math.toDegrees(angleRad);

                telemetry.addData("Angle (deg)", angleDeg);
                telemetry.addData("ServoPos", SERVO_INTERCEPT + SERVO_SLOPE * angleDeg);

                // telemetry
                telemetry.addData("Target Detected", true);

                telemetry.addData("Distance (inches)", "%.2f", distance);
                telemetry.addData("Hood Servo Pos", "%.3f", hoodPositionForTelemetry(distance));
                telemetry.addData("Flywheel RPM", "%.0f", 2200 + distance * 67);
                telemetry.addData("Flywheel RPM", "%.0f", 2200 + distance * 67);
            } else {
                telemetry.addData("Target Detected", false);
            }

            telemetry.update();
        }

        hood.stop();
        limelight.stop();
    }

    private double hoodPositionForTelemetry(double distance) {
        double h = tagHeight - shooterHeight;
        double angleRad = Math.atan((h + Math.sqrt(h * h + distance * distance)) / distance);
        double angleDeg = Math.toDegrees(angleRad);
        return SERVO_INTERCEPT + SERVO_SLOPE * angleDeg;
    }


}