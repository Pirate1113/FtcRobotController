package org.firstinspires.ftc.teamcode.testing;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Auto Aim Hood Test", group="Testing")
public class AutoAimTestOpMode extends LinearOpMode {

    private HoodAngle hood;
    private LimelightAngle limelight;

    // needs tuning of course
    private static final double LIMELIGHT_HEIGHT = 14.0;
    private static final double TAG_HEIGHT = 29.5;

    double hoodPos = 0.0;
    private static final double HOOD_STEP = 0.01;


    @Override
    public void runOpMode() {

        // hoodangle
        hood = new HoodAngle(hardwareMap, LIMELIGHT_HEIGHT, TAG_HEIGHT);

        // yes LIMELIGHT yes yez
        limelight = new LimelightAngle(hardwareMap, "limelight", LIMELIGHT_HEIGHT, TAG_HEIGHT);

        telemetry.addLine("Hood and Limelight initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (limelight.hasTarget()) {
                // get distance
                double distance = limelight.getDistanceInches(telemetry);

                // autoaim
                hood.aimFromDistance(distance);

                double stick = -gamepad1.right_stick_y; // up = positive

                hoodPos += stick * HOOD_STEP;
                hoodPos = Range.clip(hoodPos, 0.0, 1.0);
                hood.setHoodPos(hoodPos);

                // telemetry
                telemetry.addData("Target Detected", true);
                telemetry.addData("Distance (inches)", "%.2f", distance);
                telemetry.addData("Hood Servo Pos", "%.3f", hoodPositionForTelemetry(distance));
                telemetry.addData("ServoPos: ", hoodPos);
                hood.getTelemetry(telemetry);
            } else {
                telemetry.addData("Target Detected", false);
            }

            telemetry.update();
        }

        hood.stop();
        limelight.stop();
    }

    private double hoodPositionForTelemetry(double distance) {
        double h = TAG_HEIGHT - LIMELIGHT_HEIGHT;
        double angleRad = Math.atan((h + Math.sqrt(h * h + distance * distance)) / distance);
        double angleDeg = Math.toDegrees(angleRad);
        return HoodAngle.SERVO_INTERCEPT + HoodAngle.SERVO_SLOPE * angleDeg;
    }
}