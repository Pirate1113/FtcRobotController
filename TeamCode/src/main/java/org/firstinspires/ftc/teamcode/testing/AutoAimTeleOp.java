package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Auto Aim", group="Testing")
public class AutoAimTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing...");
        telemetry.update();

        // hardware
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LimelightAngle vision = new LimelightAngle(limelight, 6.0, 12.5); // Shooter height & tag offset

        //
        SwerveDrivetrain drive = SwerveDrivetrain.INSTANCE;
        drive.initialize(); //

        HoodAngle shooter = new HoodAngle(hardwareMap);

        // Start Limelight streaming
        limelight.start();

        telemetry.addLine("Limelight started");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Check if Limelight sees a target
            boolean hasTarget = vision.hasTarget();
            telemetry.addData("Has Target", hasTarget);

            if (hasTarget) {
                double yaw = vision.getYaw();         // Horizontal offset from robot forward
                double pitch = vision.getPitch();     // Vertical angle for hood
                double distance = vision.getDistance(); // Horizontal distance to target

                // --- Turn robot toward target ---
                drive.turnToYaw(yaw); // rotates robot in place toward the target

                // --- Adjust shooter ---
                shooter.setHoodFromPitch(pitch);            // hood position from pitch
                shooter.setFlywheelFromDistance(distance);  // flywheel speed from distance

                // --- Telemetry ---
                telemetry.addData("Yaw (deg)", yaw);
                telemetry.addData("Pitch (deg)", pitch);
                telemetry.addData("Distance (in)", distance);

            } else {
                // No target detected, stop drivetrain and shooter
                drive.turnToYaw(0);
                shooter.stop();
                telemetry.addLine("No target detected");
            }

            telemetry.update();
            idle();
        }
    }
}