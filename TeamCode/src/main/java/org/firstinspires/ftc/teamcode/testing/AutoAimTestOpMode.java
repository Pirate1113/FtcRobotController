package org.firstinspires.ftc.teamcode.testing;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Auto Aim Hood Test", group = "Testing")
public class AutoAimTestOpMode extends LinearOpMode {

    private HoodAngle hood;
    private LimelightAngle limelight;

<<<<<<< HEAD
    // tuning needed?!
    private static final double SHOOTER_HEIGHT_INCHES = 12.0;

    // basket tag
    private static final double TAG_HEIGHT_INCHES = 37.0;

    // things that need tuning
    private static final double CAMERA_HEIGHT_INCHES = 12.0;  // height of LL from ground
    private static final double CAMERA_PITCH_DEG = 0.0;      // LL tilt up from horizontal

    private static final double SHOOTER_TO_CAMERA_OFFSET_INCHES = 2.0; // idk if needed
=======
    // needs tuning of course
    private static final double LIMELIGHT_HEIGHT = 14.0;
    private static final double TAG_HEIGHT = 29.5;

    double hoodPos = 0.0;
    private static final double HOOD_STEP = 0.01;

>>>>>>> 58e1ccaacf0918166dce8467964b78cd2725b165

    @Override
    public void runOpMode() {

<<<<<<< HEAD
        // hood + flywheel subsystem
        hood = new HoodAngle(hardwareMap, SHOOTER_HEIGHT_INCHES, TAG_HEIGHT_INCHES);


        limelight = new LimelightAngle(
                hardwareMap,
                "limelight",           // device name in configuration
                CAMERA_HEIGHT_INCHES,
                CAMERA_PITCH_DEG,
                TAG_HEIGHT_INCHES
        );
=======
        // hoodangle
        hood = new HoodAngle(hardwareMap, LIMELIGHT_HEIGHT, TAG_HEIGHT);

        // yes LIMELIGHT yes yez
        limelight = new LimelightAngle(hardwareMap, "limelight", LIMELIGHT_HEIGHT, TAG_HEIGHT);
>>>>>>> 58e1ccaacf0918166dce8467964b78cd2725b165

        telemetry.addLine("Hood and Limelight initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (limelight.hasTarget()) {
<<<<<<< HEAD
                // distance from camera to tag
                double distanceCamera = limelight.getDistanceInches();
=======
                // get distance
                double distance = limelight.getDistanceInches(telemetry);
>>>>>>> 58e1ccaacf0918166dce8467964b78cd2725b165

                // convert to shooter-to-tag distance accounting for offset, I don't really know if we need this but sure ig
                double distanceShooter = distanceCamera + SHOOTER_TO_CAMERA_OFFSET_INCHES;

                // auto-aim
                hood.aimFromDistance(distanceShooter);

                double stick = -gamepad1.right_stick_y; // up = positive

                hoodPos += stick * HOOD_STEP;
                hoodPos = Range.clip(hoodPos, 0.0, 1.0);
                hood.setHoodPos(hoodPos);

                // telemetry
                telemetry.addData("Target Detected", true);
<<<<<<< HEAD
                telemetry.addData("Distance (cam)", "%.2f in", distanceCamera);
                telemetry.addData("Distance (shooter)", "%.2f in", distanceShooter);
                telemetry.addData("Flywheel RPM", "%.0f", 6000.0);
                telemetry.addData("Limelight ty", "%.2f deg", limelight.getPitch());
                telemetry.addData("Limelight tx", "%.2f deg", limelight.getYaw());
=======
                telemetry.addData("Distance (inches)", "%.2f", distance);
                telemetry.addData("Hood Servo Pos", "%.3f", hoodPositionForTelemetry(distance));
                telemetry.addData("ServoPos: ", hoodPos);
                hood.getTelemetry(telemetry);
>>>>>>> 58e1ccaacf0918166dce8467964b78cd2725b165
            } else {
                telemetry.addData("Target Detected", false);
            }

            telemetry.update();
        }

        hood.stop();
        limelight.stop();
    }
<<<<<<< HEAD
=======

    private double hoodPositionForTelemetry(double distance) {
        double h = TAG_HEIGHT - LIMELIGHT_HEIGHT;
        double angleRad = Math.atan((h + Math.sqrt(h * h + distance * distance)) / distance);
        double angleDeg = Math.toDegrees(angleRad);
        return HoodAngle.SERVO_INTERCEPT + HoodAngle.SERVO_SLOPE * angleDeg;
    }
>>>>>>> 58e1ccaacf0918166dce8467964b78cd2725b165
}