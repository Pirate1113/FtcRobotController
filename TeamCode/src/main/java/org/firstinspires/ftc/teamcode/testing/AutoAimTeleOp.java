package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Auto Aim")
public class AutoAimTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        Limelight3A limelight =
                hardwareMap.get(Limelight3A.class, "limelight");

        LimelightAngle vision =
                new LimelightAngle(limelight, 6.0, 12.5); // needs tuning??

        SwerveToShootingAngle turner =
                new SwerveToShootingAngle(hardwareMap);

        HoodAngle shooter =
                new HoodAngle(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (vision.hasTarget()) {
                double yaw = vision.getYaw();
                double pitch = vision.getPitch();
                double distance = vision.getDistance();

                turner.turnToTarget(yaw);
                shooter.setHoodFromPitch(pitch);
                shooter.setFlywheelFromDistance(distance);
            } else {
                turner.stop();
                shooter.stop();
            }

            telemetry.addData("Yaw", vision.getYaw());
            telemetry.addData("Pitch", vision.getPitch());
            telemetry.update();
        }
    }
}