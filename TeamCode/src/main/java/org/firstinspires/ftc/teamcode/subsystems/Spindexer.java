//package org.firstinspires.ftc.teamcode.subsystems;
//
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.subsystems.Subsystem;
//import dev.nextftc.hardware.controllable.MotorGroup;
//import dev.nextftc.hardware.impl.CRServoEx;
//import dev.nextftc.hardware.impl.MotorEx;
//import dev.nextftc.hardware.powerable.SetPower;
//
//public class Spindexer implements Subsystem {
//    public static final Spindexer INSTANCE = new Spindexer();
//    private static final double POWER_INCREMENT = 0.2;
//
//    private Spindexer() {}
//
//    private CRServoEx servoLeft;
//    private CRServoEx servoRight;
//
//    private double powerLeft = 0.0;
//    private double powerRight = 0.0;
//
//    public void initialize() {
//        servoLeft = new CRServoEx("spindexerleft");
//        servoRight = new CRServoEx("spindexerright");
//    }
//
//    // ===== SYNCHRONIZED CONTROL =====
//
//    /** Run both intake motors at full power to intake samples */
//    public Command intake() {
//        return new SetPower(intakeMotors, 1.0).named("Intake");
//    }
//
//    /** Run both intake motors in reverse to eject samples */
//    public Command outtake() {
//        return new SetPower(intakeMotors, -1.0).named("Outtake");
//    }
//
//    /** Stop both intake motors */
//    public Command stop() {
//        return new SetPower(intakeMotors, 0.0).named("Stop Intake");
//    }
//
//    /** Set both motors to a specific power */
//    public Command setPower(double power) {
//        powerLeft = power;
//        powerRight = power;
//        return new SetPower(intakeMotors, power).named("Set Intake Power");
//    }
//
//    // ===== INDEPENDENT MOTOR CONTROL =====
//
//    /** Increase left motor power by increment */
//    public Command increaseLeft() {
//        powerLeft = Math.min(1.0, powerLeft + POWER_INCREMENT);
//        return new SetPower(intakeLeft, powerLeft).named("Increase Left Power");
//    }
//
//    /** Decrease left motor power by increment */
//    public Command decreaseLeft() {
//        powerLeft = Math.max(-1.0, powerLeft - POWER_INCREMENT);
//        return new SetPower(intakeLeft, powerLeft).named("Decrease Left Power");
//    }
//
//    /** Increase right motor power by increment */
//    public Command increaseRight() {
//        powerRight = Math.min(1.0, powerRight + POWER_INCREMENT);
//        return new SetPower(intakeRight, powerRight).named("Increase Right Power");
//    }
//
//    /** Decrease right motor power by increment */
//    public Command decreaseRight() {
//        powerRight = Math.max(-1.0, powerRight - POWER_INCREMENT);
//        return new SetPower(intakeRight, powerRight).named("Decrease Right Power");
//    }
//
//    /** Set left motor to specific power */
//    public Command setLeftPower(double power) {
//        powerLeft = Math.max(-1.0, Math.min(1.0, power));
//        return new SetPower(intakeLeft, powerLeft).named("Set Left Power");
//    }
//
//    /** Set right motor to specific power */
//    public Command setRightPower(double power) {
//        powerRight = Math.max(-1.0, Math.min(1.0, power));
//        return new SetPower(intakeRight, powerRight).named("Set Right Power");
//    }
//
//    // ===== GETTERS FOR TELEMETRY =====
//
//    public double getLeftPower() {
//        return powerLeft;
//    }
//
//    public double getRightPower() {
//        return powerRight;
//    }
//}