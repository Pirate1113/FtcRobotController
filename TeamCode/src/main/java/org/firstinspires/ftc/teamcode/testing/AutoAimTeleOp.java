package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Auto Aim")
public class AutoAimTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing...");
        telemetry.update();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        LimelightAngle vision = new LimelightAngle(limelight, 6.0, 12.5);

        // Replace with your actual SwerveDrivetrain initialization
        SwerveDrivetrain drivetrain = new SwerveDrivetrain(
                hardwareMap.get(DcMotorEx.class,"frMotor"),
                hardwareMap.get(CRServo.class,"frSteer"),
                hardwareMap.get(AnalogInput.class,"frEncoder"),

                hardwareMap.get(DcMotorEx.class,"brMotor"),
                hardwareMap.get(CRServo.class,"brSteer"),
                hardwareMap.get(AnalogInput.class,"brEncoder"),

                hardwareMap.get(DcMotorEx.class,"blMotor"),
                hardwareMap.get(CRServo.class,"blSteer"),
                hardwareMap.get(AnalogInput.class,"blEncoder"),

                hardwareMap.get(DcMotorEx.class,"flMotor"),
                hardwareMap.get(CRServo.class,"flSteer"),
                hardwareMap.get(AnalogInput.class,"flEncoder")
        );

        HoodAngle shooter = new HoodAngle(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if (vision.hasTarget()) {
                double yaw = vision.getYaw();
                double pitch = vision.getPitch();
                double distance = vision.getDistance();

                drivetrain.turnToYaw(yaw);
                shooter.setHoodFromPitch(pitch);
                shooter.setFlywheelFromDistance(distance);

                telemetry.addData("Has Target", true);
                telemetry.addData("Yaw", yaw);
                telemetry.addData("Pitch", pitch);
                telemetry.addData("Distance", distance);
            } else {
                drivetrain.stop();
                shooter.stop();
                telemetry.addData("Has Target", false);
            }

            telemetry.update();
        }
    }
}