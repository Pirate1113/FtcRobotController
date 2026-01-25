package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Auto Aim Hood Test", group = "Testing")
public class AutoAimTestOpMode extends LinearOpMode {

    private HoodAngle hood;
    private LimelightAngle limelight;

    // tuning needed?!
    private static final double SHOOTER_HEIGHT_INCHES = 12.0;

    // basket tag
    private static final double TAG_HEIGHT_INCHES = 37.0;

    // things that need tuning
    private static final double CAMERA_HEIGHT_INCHES = 12.0;  // height of LL from ground
    private static final double CAMERA_PITCH_DEG = 0.0;      // LL tilt up from horizontal

    private static final double SHOOTER_TO_CAMERA_OFFSET_INCHES = 2.0; // idk if needed

    @Override
    public void runOpMode() {

        // hood + flywheel subsystem
        hood = new HoodAngle(hardwareMap, SHOOTER_HEIGHT_INCHES, TAG_HEIGHT_INCHES);


        limelight = new LimelightAngle(
                hardwareMap,
                "limelight",
                CAMERA_HEIGHT_INCHES,
                CAMERA_PITCH_DEG,
                TAG_HEIGHT_INCHES
        );

        telemetry.addLine("Hood and Limelight initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (limelight.hasTarget()) {
                // distance from camera to tag
                double distanceCamera = limelight.getDistanceInches();

                // convert to shooter-to-tag distance accounting for offset, I don't really know if we need this but sure ig
                double distanceShooter = distanceCamera + SHOOTER_TO_CAMERA_OFFSET_INCHES;

                // auto-aim
                hood.aimFromDistance(distanceShooter);

                // telemetry
                telemetry.addData("Target Detected", true);
                telemetry.addData("Distance (cam)", "%.2f in", distanceCamera);
                telemetry.addData("Distance (shooter)", "%.2f in", distanceShooter);
                telemetry.addData("Flywheel RPM", "%.0f", 6000.0);
                telemetry.addData("Limelight ty", "%.2f deg", limelight.getPitch());
                telemetry.addData("Limelight tx", "%.2f deg", limelight.getYaw());
            } else {
                telemetry.addData("Target Detected", false);
            }

            telemetry.update();
        }

        hood.stop();
        limelight.stop();
    }
}