package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private static final double POWER = 0.9;

    private Intake() {}

    private MotorEx intakeFront;
    private MotorEx intakeBack;
    private double powerFront = 0;
    private double powerBack = 0;

    public void initialize() {
        intakeFront = new MotorEx("f_intake").reversed().floatMode();
        intakeBack = new MotorEx("b_intake").floatMode();
    }


    // ===== INDEPENDENT MOTOR CONTROL =====
    public Command moveFront = new InstantCommand(() -> {
        powerFront = POWER;
        intakeFront.getMotor().setPower(-powerFront);
    });
    public Command moveBack = new InstantCommand(() -> {
        powerBack = POWER;
        intakeBack.getMotor().setPower(-powerBack);
    });
    public Command stopFront = new InstantCommand(() -> {
        powerFront = 0;
        intakeFront.getMotor().setPower(powerFront);
    });
    public Command stopBack = new InstantCommand(() -> {
        powerBack = 0;
        intakeBack.getMotor().setPower(-powerBack);
    });

    public double getFrontPower() {
        return powerFront;
    }

    public double getBackPower() {
        return powerBack;
    }
}
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//
//@TeleOp
//@Config
//public class Intake extends OpMode {
//
//    DcMotorEx intakeleft;
//    DcMotorEx intakeright;
//
//    double powerLeft = 0.0;
//    double powerRight = 0.0;
//
//    boolean previousDpadUp;
//    boolean previousDpadDown;
//    boolean previousX;
//    boolean previousB;
//
//
//    @Override
//    public void init() {
//        intakeleft = hardwareMap.get(DcMotorEx.class, "intakeleft");
//        intakeright = hardwareMap.get(DcMotorEx.class, "intakeright");
//
//        intakeleft.setDirection(DcMotor.Direction.REVERSE);
//        intakeleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        intakeright.setDirection(DcMotor.Direction.FORWARD);
//        intakeright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//    }
//
//    public void loop() {
//        // Left intake control with dpad
//        if (gamepad1.dpad_up && !previousDpadUp) {
//            powerLeft += 0.1;
//        } else if (gamepad1.dpad_down && !previousDpadDown) {
//            powerLeft -= 0.1;
//        }
//
//        // Right intake control with X (square) and B (circle)
//        if (gamepad1.x && !previousX) {
//            powerRight += 0.1;
//        } else if (gamepad1.b && !previousB) {
//            powerRight -= 0.1;
//        }
//
//        // Set motor powers
//        intakeleft.setPower(powerLeft);
//        intakeright.setPower(powerRight);
//
//        // Update previous button states
//        previousDpadUp = gamepad1.dpad_up;
//        previousDpadDown = gamepad1.dpad_down;
//        previousX = gamepad1.x;
//        previousB = gamepad1.b;
//
//        // Telemetry for debugging
//        telemetry.addData("Left Intake Power", powerLeft);
//        telemetry.addData("Right Intake Power", powerRight);
//        telemetry.addData("previousX", previousX);
//        telemetry.addData("previousB", previousB);
//        telemetry.addData("previousDpadUp", previousDpadUp);
//        telemetry.addData("previousDpadDown", previousDpadDown);
//        telemetry.update();
//
//    }
//}