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
    public static final double LLHeight = 14;
    public static final double shooterHeight = 12;
    public static final double tagHeight = 29.5;

    @Override
    public void runOpMode() {

        // hoodangle
        hood = new HoodAngle(hardwareMap, LLHeight, tagHeight);

        // yes LIMELIGHT yes yez
        limelight = new LimelightAngle(hardwareMap, "limelight", LLHeight, tagHeight);

        telemetry.addLine("Hood and Limelight initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (limelight.hasTarget()) {
                telemetry.addData("Target Detected", true);

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
                double servoPos = angleDeg * 13/255;

                // telemetry
                telemetry.addData("Hood angle: ", angleDeg);
                telemetry.addData("ServoPos: ", servoPos);
                telemetry.addData("Distance (inches)", "%.2f", distance);
                telemetry.addData("Fake RPM idk if we need", "%.0f", 2200 + distance * 67);
                telemetry.addData("Flywheel REAL RPM", "%.0f", hood.getFlywheelRpm());

            } else {
                telemetry.addData("Target Detected", false);
                hood.stop();
            }

            telemetry.update();
        }

        hood.stop();
        limelight.stop();
    }




}