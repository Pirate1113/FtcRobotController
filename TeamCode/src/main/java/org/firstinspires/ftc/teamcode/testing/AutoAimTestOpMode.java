package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="HoodAngle Test", group="Testing")
public class AutoAimTestOpMode extends LinearOpMode {

    private HoodAngle hood;

    @Override
    public void runOpMode() {

        hood = new HoodAngle(hardwareMap);

        telemetry.addLine("HoodAngle initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double stickY = -gamepad1.left_stick_y;
            double distanceInches = (stickY + 1) / 2 * 60;

            // aiming
            hood.aimFromDistance(distanceInches);

            // telemetry
            telemetry.addData("Stick Y", stickY);
            telemetry.addData("Distance (in)", distanceInches);
            telemetry.addData("Hood Servo Pos", hoodPositionFromDistanceForTelemetry(distanceInches));
            telemetry.addData("Flywheel RPM", 2200 + distanceInches * 67);
            telemetry.update();
        }

        hood.stop();
    }

    private double hoodPositionFromDistanceForTelemetry(double distance) {
        double h = HoodAngle.TARGET_HEIGHT - HoodAngle.SHOOTER_HEIGHT;
        double angleRad = Math.atan((h + Math.sqrt(h * h + distance * distance)) / distance);
        double angleDeg = Math.toDegrees(angleRad);
        return HoodAngle.SERVO_INTERCEPT + HoodAngle.SERVO_SLOPE * angleDeg;
    }
}