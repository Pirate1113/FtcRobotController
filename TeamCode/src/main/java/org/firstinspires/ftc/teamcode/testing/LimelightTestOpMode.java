//package org.firstinspires.ftc.teamcode.testing;
//
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name = "Limelight Test", group = "Testing")
//public class LimelightTestOpMode extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//
//        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
//        limelight.start();
//
//        LimelightAngle vision = new LimelightAngle(
//                limelight,
//                6.0,    // shooter height
//                12.5    // tag offset
//        );
//
//        telemetry.addLine("Limelight initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            telemetry.addData("Has Target", vision.hasTarget());
//            telemetry.addData("Yaw (tx)", vision.getYaw());
//            telemetry.addData("Pitch (ty)", vision.getPitch());
//            telemetry.addData("Distance (in)", vision.getDistance());
//            telemetry.update();
//        }
//    }
//}