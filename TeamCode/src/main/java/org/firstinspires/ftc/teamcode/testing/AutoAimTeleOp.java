package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Auto Aim")
public class AutoAimTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing...");
        telemetry.update();

        // ---- Hardware ----
        Limelight3A limelight =
                hardwareMap.get(Limelight3A.class, "limelight");

        LimelightAngle vision =
                new LimelightAngle(limelight, 6.0, 12.5);

        SwerveToShootingAngle turner =
                new SwerveToShootingAngle(hardwareMap);

        HoodAngle shooter =
                new HoodAngle(hardwareMap);

        // ---- START LIMELIGHT ----
        limelight.start();

        telemetry.addLine("Limelight started");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean hasTarget = vision.hasTarget();

            telemetry.addData("Has Target", hasTarget);

            if (hasTarget) {
                double yaw = vision.getYaw();
                double pitch = vision.getPitch();
                double distance = vision.getDistance();

                turner.turnToTarget(yaw);
                shooter.setHoodFromPitch(pitch);
                shooter.setFlywheelFromDistance(distance);

                telemetry.addData("Yaw (deg)", yaw);
                telemetry.addData("Pitch (deg)", pitch);
                telemetry.addData("Distance (in)", distance);
                telemetry.addData("Raw Result", limelight.getLatestResult());

            } else {
                turner.stop();
                shooter.stop();
                telemetry.addLine("No target detected");
            }

            telemetry.update();
            idle();
        }
    }
}