package org.firstinspires.ftc.teamcode.testing;
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
                double servoPos = angleDeg * 8.125/255;

                // telemetry
                telemetry.addData("Hood angle: ", angleDeg);
                telemetry.addData("ServoPos: ", servoPos);
                telemetry.addData("Distance (inches)", "%.2f", distance);
            } else {
                telemetry.addData("Target Detected", false);
            }

            telemetry.update();
        }

        limelight.stop();
    }




}