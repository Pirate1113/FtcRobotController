package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Test", group = "Testing")
public class LimelightTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        LimelightAngle vision = new LimelightAngle(
                limelight,
                6.0,
                12.5
        );

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Has Target", vision.hasTarget());
            telemetry.addData("Yaw", vision.getYaw());
            telemetry.addData("Pitch", vision.getPitch());
            telemetry.addData("Distance", vision.getDistance());
            telemetry.update();
        }
    }
}
