package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private static final double POWER_INCREMENT = 0.2;

    private Intake() {}

    private MotorEx intakeLeft;
    private MotorEx intakeRight;
    private MotorGroup intakeMotors;

    private double powerLeft = 0.0;
    private double powerRight = 0.0;

    public void initialize() {
        intakeLeft = new MotorEx("intakeleft").reversed().floatMode();
        intakeRight = new MotorEx("intakeright").floatMode();
        intakeMotors = new MotorGroup(intakeRight, intakeLeft);
    }

    // ===== SYNCHRONIZED CONTROL =====

    /** Run both intake motors at full power to intake samples */
    public Command intake() {
        return new SetPower(intakeMotors, 1.0).named("Intake");
    }

    /** Run both intake motors in reverse to eject samples */
    public Command outtake() {
        return new SetPower(intakeMotors, -1.0).named("Outtake");
    }

    /** Stop both intake motors */
    public Command stop() {
        return new SetPower(intakeMotors, 0.0).named("Stop Intake");
    }

    /** Set both motors to a specific power */
    public Command setPower(double power) {
        powerLeft = power;
        powerRight = power;
        return new SetPower(intakeMotors, power).named("Set Intake Power");
    }

    // ===== INDEPENDENT MOTOR CONTROL =====

    /** Increase left motor power by increment */
    public Command increaseLeft() {
        powerLeft = Math.min(1.0, powerLeft + POWER_INCREMENT);
        return new SetPower(intakeLeft, powerLeft).named("Increase Left Power");
    }

    /** Decrease left motor power by increment */
    public Command decreaseLeft() {
        powerLeft = Math.max(-1.0, powerLeft - POWER_INCREMENT);
        return new SetPower(intakeLeft, powerLeft).named("Decrease Left Power");
    }

    /** Increase right motor power by increment */
    public Command increaseRight() {
        powerRight = Math.min(1.0, powerRight + POWER_INCREMENT);
        return new SetPower(intakeRight, powerRight).named("Increase Right Power");
    }

    /** Decrease right motor power by increment */
    public Command decreaseRight() {
        powerRight = Math.max(-1.0, powerRight - POWER_INCREMENT);
        return new SetPower(intakeRight, powerRight).named("Decrease Right Power");
    }

    /** Set left motor to specific power */
    public Command setLeftPower(double power) {
        powerLeft = Math.max(-1.0, Math.min(1.0, power));
        return new SetPower(intakeLeft, powerLeft).named("Set Left Power");
    }

    /** Set right motor to specific power */
    public Command setRightPower(double power) {
        powerRight = Math.max(-1.0, Math.min(1.0, power));
        return new SetPower(intakeRight, powerRight).named("Set Right Power");
    }

    // ===== GETTERS FOR TELEMETRY =====

    public double getLeftPower() {
        return powerLeft;
    }

    public double getRightPower() {
        return powerRight;
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